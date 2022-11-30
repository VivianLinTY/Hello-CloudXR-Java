/*
 * Copyright 2017 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "hello_ar_application.h"

#include <android/asset_manager.h>
#include <array>
#include <mutex>
#include <EGL/egl.h>

#include "oboe/Oboe.h"

#include "util.h"

#include "CloudXRClient.h"
#include "CloudXRInputEvents.h"
#include "CloudXRClientOptions.h"
#include "CloudXRMatrixHelpers.h"

#ifndef CHECK_NOTIFY_STATUS
#define CHECK_NOTIFY_STATUS(stat, terminate)                                               \
  if (stat!=AR_SUCCESS) {                                                                 \
    NotifyUserError(stat, __FILE__, __LINE__, terminate);\
  }
#endif  // CHECK

namespace hello_ar {
    namespace {
        const glm::vec3 kWhite = {255, 255, 255};
    }  // namespace

    class ARLaunchOptions : public CloudXR::ClientOptions {
    public:
        bool using_env_lighting_;
        float res_factor_;

        ARLaunchOptions() :
                ClientOptions(),
                using_env_lighting_(true), // default ON
                // default to 0.75 reduced size, as many devices can't handle full throughput.
                // 0.75 chosen as WAR value for steamvr buffer-odd-size bug, works on galaxytab s6 + pixel 2
                res_factor_(0.75f) {
            AddOption("env-lighting", "el", true,
                      "Send client environment lighting data to server.  1 enables, 0 disables.",
                      HANDLER_LAMBDA_FN {
                          if (tok == "1") {
                              using_env_lighting_ = true;
                          } else if (tok == "0") {
                              using_env_lighting_ = false;
                          }
                          return ParseStatus_Success;
                      });
            AddOption("res-factor", "rf", true,
                      "Adjust client resolution sent to server, reducing res by factor. Range [0.5-1.0].",
                      HANDLER_LAMBDA_FN {
                          float factor = std::stof(tok);
                          if (factor >= 0.5f && factor <= 1.0f)
                              res_factor_ = factor;
                          LOGI("Resolution factor = %0.2f", res_factor_);
                          return ParseStatus_Success;
                      });
        }
    };


    class HelloArApplication::CloudXRClient : public oboe::AudioStreamDataCallback {
    public:
        ~CloudXRClient() {
            Teardown();
        }

        // CloudXR interface callbacks
        void TriggerHaptic(const cxrHapticFeedback *) {}

        void GetTrackingState(cxrVRTrackingState *state) {
            *state = {};

            state->hmd.pose.poseIsValid = cxrTrue;
            state->hmd.pose.deviceIsConnected = cxrTrue;
            state->hmd.pose.trackingResult = cxrTrackingResult_Running_OK;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                const int idx = current_idx_ == 0 ?
                                kQueueLen - 1 : (current_idx_ - 1) % kQueueLen;
                cxrMatrixToVecQuat(pose_matrix_ + idx, &(state->hmd.pose.position),
                                   &(state->hmd.pose.rotation));
            }
        }

        cxrBool RenderAudio(const cxrAudioFrame *audioFrame) {
            if (!playback_stream_ || exiting_) {
                return cxrFalse;
            }

            const uint32_t timeout = audioFrame->streamSizeBytes / CXR_AUDIO_BYTES_PER_MS;
            const uint32_t numFrames = timeout * CXR_AUDIO_SAMPLING_RATE / 1000;
            playback_stream_->write(audioFrame->streamBuffer, numFrames,
                                    timeout * oboe::kNanosPerMillisecond);

            return cxrTrue;
        }

        // AudioStreamDataCallback interface
        oboe::DataCallbackResult onAudioReady(oboe::AudioStream *oboeStream,
                                              void *audioData, int32_t numFrames) {
            if (!recording_stream_ || exiting_) {
                return oboe::DataCallbackResult::Stop;
            }
            cxrAudioFrame recordedFrame{};
            recordedFrame.streamBuffer = (int16_t *) audioData;
            recordedFrame.streamSizeBytes =
                    numFrames * CXR_AUDIO_CHANNEL_COUNT * CXR_AUDIO_SAMPLE_SIZE;
            cxrSendAudio(cloudxr_receiver_, &recordedFrame);

            return oboe::DataCallbackResult::Continue;
        }

        cxrDeviceDesc GetDeviceDesc() {
            device_desc_.deliveryType = cxrDeliveryType_Mono_RGBA;
            device_desc_.width = stream_width_;
            device_desc_.height = stream_height_;
            device_desc_.maxResFactor = 1.0f; // leave alone, don't extra oversample on server.
            device_desc_.fps = static_cast<float>(fps_);
            device_desc_.ipd = 0.064f;
            device_desc_.predOffset = 0.02f;
            device_desc_.receiveAudio = launch_options_.mReceiveAudio;
            device_desc_.sendAudio = launch_options_.mSendAudio;
            device_desc_.disablePosePrediction = false;
            device_desc_.angularVelocityInDeviceSpace = false;
            device_desc_.foveatedScaleFactor = 0; // ensure no foveation for AR.

            return device_desc_;
        }

        cxrError Connect() {
            if (cloudxr_receiver_)
                return cxrError_Success; // already connected, no error? TODO

            LOGI("Connecting to CloudXR at %s...", launch_options_.mServerIP.c_str());

            cxrGraphicsContext context{cxrGraphicsContext_GLES};
            context.egl.display = eglGetCurrentDisplay();
            context.egl.context = eglGetCurrentContext();

            auto device_desc = GetDeviceDesc();

            cxrClientCallbacks clientProxy = {0};
            clientProxy.GetTrackingState = [](void *context, cxrVRTrackingState *trackingState) {
                return reinterpret_cast<CloudXRClient *>(context)->GetTrackingState(trackingState);
            };
            clientProxy.TriggerHaptic = [](void *context, const cxrHapticFeedback *haptic) {
                return reinterpret_cast<CloudXRClient *>(context)->TriggerHaptic(haptic);
            };
            clientProxy.RenderAudio = [](void *context, const cxrAudioFrame *audioFrame) {
                return reinterpret_cast<CloudXRClient *>(context)->RenderAudio(audioFrame);
            };

            if (device_desc.receiveAudio) {
                // Initialize audio playback
                oboe::AudioStreamBuilder playback_stream_builder;
                playback_stream_builder.setDirection(oboe::Direction::Output);
                // On some platforms oboe::PerformanceMode::LowLatency leads to stutter during playback of
                // audio received from the server, using oboe::PerformanceMode::None as shown below can help:
                // playback_stream_builder.setPerformanceMode(oboe::PerformanceMode::None);
                playback_stream_builder.setPerformanceMode(oboe::PerformanceMode::LowLatency);
                playback_stream_builder.setSharingMode(oboe::SharingMode::Exclusive);
                playback_stream_builder.setFormat(oboe::AudioFormat::I16);
                playback_stream_builder.setChannelCount(oboe::ChannelCount::Stereo);
                playback_stream_builder.setSampleRate(CXR_AUDIO_SAMPLING_RATE);

                oboe::Result r = playback_stream_builder.openStream(playback_stream_);
                if (r != oboe::Result::OK) {
                    LOGE("Failed to open playback stream. Error: %s", oboe::convertToText(r));
                    //return; // for now continue to run...
                } else {
                    int bufferSizeFrames = playback_stream_->getFramesPerBurst() * 2;
                    r = playback_stream_->setBufferSizeInFrames(bufferSizeFrames);
                    if (r != oboe::Result::OK) {
                        LOGE("Failed to set playback stream buffer size to: %d. Error: %s",
                             bufferSizeFrames, oboe::convertToText(r));
                        //return; // for now continue to run...
                    } else {
                        r = playback_stream_->start();
                        if (r != oboe::Result::OK) {
                            LOGE("Failed to start playback stream. Error: %s",
                                 oboe::convertToText(r));
                            //return; // for now continue to run...
                        }
                    }
                }

                // if there was an error setting up, turn off receiving audio for this connection.
                if (r != oboe::Result::OK) {
                    device_desc.receiveAudio = false;
                    launch_options_.mReceiveAudio = false;
                    if (playback_stream_) {
                        playback_stream_->close();
                        playback_stream_.reset();
                    }
                }
            }

            if (device_desc.sendAudio) {
                // Initialize audio recording
                oboe::AudioStreamBuilder recording_stream_builder;
                recording_stream_builder.setDirection(oboe::Direction::Input);
                recording_stream_builder.setPerformanceMode(oboe::PerformanceMode::LowLatency);
                recording_stream_builder.setSharingMode(oboe::SharingMode::Exclusive);
                recording_stream_builder.setFormat(oboe::AudioFormat::I16);
                recording_stream_builder.setChannelCount(oboe::ChannelCount::Stereo);
                recording_stream_builder.setSampleRate(CXR_AUDIO_SAMPLING_RATE);
                recording_stream_builder.setInputPreset(oboe::InputPreset::VoiceCommunication);
                recording_stream_builder.setDataCallback(this);

                oboe::Result r = recording_stream_builder.openStream(recording_stream_);
                if (r != oboe::Result::OK) {
                    LOGE("Failed to open recording stream. Error: %s", oboe::convertToText(r));
                    //return; // for now continue to run...
                } else {
                    r = recording_stream_->start();
                    if (r != oboe::Result::OK) {
                        LOGE("Failed to start recording stream. Error: %s", oboe::convertToText(r));
                        //return; // for now continue to run...
                    }
                }

                // if there was an error setting up, turn off sending audio for this connection.
                if (r != oboe::Result::OK) {
                    device_desc.sendAudio = false;
                    launch_options_.mSendAudio = false;
                    if (recording_stream_) {
                        recording_stream_->close();
                        recording_stream_.reset();
                    }
                }
            }

            LOGI("Audio support: receive [%s], send [%s]", device_desc.receiveAudio ? "on" : "off",
                 device_desc.sendAudio ? "on" : "off");

            cxrReceiverDesc desc = {0};
            desc.requestedVersion = CLOUDXR_VERSION_DWORD;
            desc.deviceDesc = device_desc;
            desc.clientCallbacks = clientProxy;
            desc.clientContext = this;
            desc.shareContext = &context;
            desc.numStreams = CXR_NUM_VIDEO_STREAMS_XR;
            desc.receiverMode = cxrStreamingMode_XR;
            desc.debugFlags = launch_options_.mDebugFlags;
            desc.logMaxSizeKB = CLOUDXR_LOG_MAX_DEFAULT;
            desc.logMaxAgeDays = CLOUDXR_LOG_MAX_DEFAULT;
            cxrError err = cxrCreateReceiver(&desc, &cloudxr_receiver_);
            if (err != cxrError_Success) {
                LOGE("Failed to create CloudXR receiver. Error %d, %s.", err, cxrErrorString(err));
                return err;
            }

            cxrConnectionDesc connectionDesc = {};
            connectionDesc.async = cxrFalse;
            connectionDesc.maxVideoBitrateKbps = launch_options_.mMaxVideoBitrate;
            connectionDesc.clientNetwork = launch_options_.mClientNetwork;
            connectionDesc.topology = launch_options_.mTopology;
            err = cxrConnect(cloudxr_receiver_, launch_options_.mServerIP.c_str(), &connectionDesc);
            if (err != cxrError_Success) {
                LOGE("Failed to connect to CloudXR server at %s. Error %d, %s.",
                     launch_options_.mServerIP.c_str(), (int) err, cxrErrorString(err));
                Teardown();
                return err;
            }

            // else, good to go.
            LOGI("Receiver created!");

            // AR shouldn't have an arena, should it?  Maybe something large?
            //LOGI("Setting default 1m radius arena boundary.", result);
            //cxrSetArenaBoundary(Receiver, 10.f, 0, 0);

            return cxrError_Success;
        }

        void Teardown() {
            if (playback_stream_) {
                playback_stream_->close();
                playback_stream_.reset();
            }

            if (recording_stream_) {
                recording_stream_->close();
                recording_stream_.reset();
            }

            if (cloudxr_receiver_) {
                LOGI("Tearing down CloudXR...");
                cxrDestroyReceiver(cloudxr_receiver_);
                cloudxr_receiver_ = nullptr;
            }
        }

        bool IsRunning() const {
            return cloudxr_receiver_;
        }

        void SetPoseMatrix(const glm::mat4 &pose_mat) {
            std::lock_guard<std::mutex> lock(state_mutex_);

            auto &pose_matrix = pose_matrix_[current_idx_];

            pose_matrix.m[0][0] = pose_mat[0][0];
            pose_matrix.m[0][1] = pose_mat[1][0];
            pose_matrix.m[0][2] = pose_mat[2][0];
            pose_matrix.m[0][3] = pose_mat[3][0];
            pose_matrix.m[1][0] = pose_mat[0][1];
            pose_matrix.m[1][1] = pose_mat[1][1];
            pose_matrix.m[1][2] = pose_mat[2][1];
            pose_matrix.m[1][3] = pose_mat[3][1];
            pose_matrix.m[2][0] = pose_mat[0][2];
            pose_matrix.m[2][1] = pose_mat[1][2];
            pose_matrix.m[2][2] = pose_mat[2][2];
            pose_matrix.m[2][3] = pose_mat[3][2];

            current_idx_ = (current_idx_ + 1) % kQueueLen;
        }

        void SetProjectionMatrix(const glm::mat4 &projection) {
            if (fabsf(projection[2][0]) > 0.0001f) {
                // Non-symmetric projection
                const float oneOver00 = 1.f / projection[0][0];
                const float l = -(1.f - projection[2][0]) * oneOver00;
                const float r = 2.f * oneOver00 + l;

                const float oneOver11 = 1.f / projection[1][1];
                const float b = -(1.f - projection[2][1]) * oneOver11;
                const float t = 2.f * oneOver11 + b;

                device_desc_.proj[0][0] = l;
                device_desc_.proj[0][1] = r;
                device_desc_.proj[0][2] = -t;
                device_desc_.proj[0][3] = -b;
            } else {
                // Symmetric projection
                device_desc_.proj[0][0] = -1.f / projection[0][0];
                device_desc_.proj[0][1] = -device_desc_.proj[0][0];
                device_desc_.proj[0][2] = -1.f / projection[1][1];
                device_desc_.proj[0][3] = -device_desc_.proj[0][2];
            }

            device_desc_.proj[1][0] = device_desc_.proj[0][0];
            device_desc_.proj[1][1] = device_desc_.proj[0][1];

            // Disable right eye rendering
            device_desc_.proj[1][2] = 0;
            device_desc_.proj[1][3] = 0;

            LOGI("Proj: %f %f %f %f", device_desc_.proj[0][0], device_desc_.proj[0][1],
                 device_desc_.proj[0][2], device_desc_.proj[0][3]);
        }

        void SetFps(int fps) {
            fps_ = fps;
        }

        int DetermineOffset() const {
            for (int offset = 0; offset < kQueueLen; offset++) {
                const int idx = current_idx_ < offset ?
                                kQueueLen + (current_idx_ - offset) % kQueueLen :
                                (current_idx_ - offset) % kQueueLen;

                const auto &pose_matrix = pose_matrix_[idx];

                int notMatch = 0;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 4; j++) {
                        if (fabsf(pose_matrix.m[i][j] - framesLatched_.poseMatrix.m[i][j]) >=
                            0.0001f)
                            notMatch++;
                    }
                }
                if (0 == notMatch) // then matrices are close enough to qualify as equal
                    return offset;
            }

            return 0;
        }

        cxrError Latch() {
            if (latched_) {
                return cxrError_Success;
            }

            if (!IsRunning()) {
                return cxrError_Receiver_Not_Running;
            }

            // Fetch the frame
            const uint32_t timeout_ms = 150;
            cxrError status = cxrLatchFrame(cloudxr_receiver_, &framesLatched_,
                                            cxrFrameMask_All, timeout_ms);

            if (status != cxrError_Success) {
                LOGI("CloudXR frame is not available!");
                return status;
            }

            latched_ = true;
            return cxrError_Success;
        }

        void Release() {
            if (!latched_) {
                return;
            }

            cxrReleaseFrame(cloudxr_receiver_, &framesLatched_);
            latched_ = false;
        }

        void Render(const float color_correction[4]) {
            if (!IsRunning() || !latched_) {
                return;
            }

            cxrBlitFrame(cloudxr_receiver_, &framesLatched_, cxrFrameMask_All);
        }

        void Stats() {
            // Log connection stats every 3 seconds
            const int STATS_INTERVAL_SEC = 3;
            frames_until_stats_--;
            if (frames_until_stats_ <= 0 &&
                cxrGetConnectionStats(cloudxr_receiver_, &stats_) == cxrError_Success) {
                // Capture the key connection statistics
                char statsString[64] = {0};
                snprintf(statsString, 64, "FPS: %6.1f    Bitrate (kbps): %5d    Latency (ms): %3d",
                         stats_.framesPerSecond, stats_.bandwidthUtilizationKbps,
                         stats_.roundTripDelayMs);

                // Turn the connection quality into a visual representation along the lines of a signal strength bar
                char qualityString[64] = {0};
                snprintf(qualityString, 64, "Connection quality: [%c%c%c%c%c]",
                         stats_.quality >= cxrConnectionQuality_Bad ? '#' : '_',
                         stats_.quality >= cxrConnectionQuality_Poor ? '#' : '_',
                         stats_.quality >= cxrConnectionQuality_Fair ? '#' : '_',
                         stats_.quality >= cxrConnectionQuality_Good ? '#' : '_',
                         stats_.quality == cxrConnectionQuality_Excellent ? '#' : '_');

                // There could be multiple reasons for low quality however we show only the most impactful to the end user here
                char reasonString[64] = {0};
                if (stats_.quality <= cxrConnectionQuality_Fair) {
                    if (stats_.qualityReasons == cxrConnectionQualityReason_EstimatingQuality) {
                        snprintf(reasonString, 64, "Reason: Estimating quality");
                    } else if (stats_.qualityReasons & cxrConnectionQualityReason_HighLatency) {
                        snprintf(reasonString, 64, "Reason: High Latency (ms): %3d",
                                 stats_.roundTripDelayMs);
                    } else if (stats_.qualityReasons & cxrConnectionQualityReason_LowBandwidth) {
                        snprintf(reasonString, 64, "Reason: Low Bandwidth (kbps): %5d",
                                 stats_.bandwidthAvailableKbps);
                    } else if (stats_.qualityReasons & cxrConnectionQualityReason_HighPacketLoss) {
                        if (stats_.totalPacketsLost == 0) {
                            snprintf(reasonString, 64, "Reason: High Packet Loss (Recoverable)");
                        } else {
                            snprintf(reasonString, 64, "Reason: High Packet Loss (%%): %3.1f",
                                     100.0f * stats_.totalPacketsLost /
                                     stats_.totalPacketsReceived);
                        }
                    }
                }

                LOGI("%s    %s    %s", statsString, qualityString, reasonString);
                frames_until_stats_ = (int) stats_.framesPerSecond * STATS_INTERVAL_SEC;
            }
        }

        void UpdateLightProps(const float primaryDirection[3], const float primaryIntensity[3],
                              const float ambient_spherical_harmonics[27]) {
            cxrLightProperties lightProperties;

            for (uint32_t n = 0; n < 3; n++) {
                lightProperties.primaryLightColor.v[n] = primaryIntensity[n];
                lightProperties.primaryLightDirection.v[n] = primaryDirection[n];
            }

            for (uint32_t n = 0; n < CXR_MAX_AMBIENT_LIGHT_SH * 3; n++) {
                lightProperties.ambientLightSh[n / 3].v[n % 3] = ambient_spherical_harmonics[n];
            }

            cxrSendLightProperties(cloudxr_receiver_, &lightProperties);
        }


        bool Init() {
            return true;
        }

        bool HandleLaunchOptions(std::string &cmdline) {
            // first, try to read "command line in a text file"
            launch_options_.ParseFile("/sdcard/CloudXRLaunchOptions.txt");
            // next, process actual 'commandline' args -- overrides any prior values
            LOGI("Parsing commandline string: %s", cmdline.c_str());
            launch_options_.ParseString(cmdline);

            // we log error here if no server (if have no 'input UI', we have no other source)
            if (launch_options_.mServerIP.empty())
                LOGE("No server IP specified yet to connect to.");

            return true;
        }

        void SetArgs(const std::string &args) {
            LOGI("App args: %s.", args.c_str());
            launch_options_.ParseString(args);
        }

        std::string GetServerAddr() {
            return launch_options_.mServerIP;
        }

        bool GetUseEnvLighting() {
            return launch_options_.using_env_lighting_;
        }

        // this is used to tell the client what the display/surface resolution is.
        // here, we can apply a factor to reduce what we tell the server our desired
        // video resolution should be.
        void SetStreamRes(uint32_t w, uint32_t h, uint32_t orientation) {
            // in portrait modes we want width to be smaller dimension
            if (w > h && (orientation == 0 || orientation == 2)) {
                std::swap(w, h);
            }

            // apply the res factor to width and height, and make sure they are even for stream res.
            stream_width_ = ((uint32_t) round((float) w * launch_options_.res_factor_)) & ~1;
            stream_height_ = ((uint32_t) round((float) h * launch_options_.res_factor_)) & ~1;
            LOGI("SetStreamRes: Display res passed = %dx%d", w, h);
            LOGI("SetStreamRes: Stream res set = %dx%d [factor %0.2f]", stream_width_,
                 stream_height_, launch_options_.res_factor_);
        }

        // Send a touch event along to the server/host application
        void HandleTouch(float x, float y) {
            if (!IsRunning()) return;

            cxrInputEvent input;
            input.type = cxrInputEventType_Touch;
            input.event.touchEvent.type = cxrTouchEventType_FINGERUP;
            input.event.touchEvent.x = x;
            input.event.touchEvent.y = y;
            cxrSendInputEvent(cloudxr_receiver_, &input);
        }

        const ARLaunchOptions &GetLaunchOptions() {
            return launch_options_;
        }

    private:
        static constexpr int kQueueLen = 16; //Same to background render

        cxrReceiverHandle cloudxr_receiver_ = nullptr;

        ARLaunchOptions launch_options_;

        uint32_t stream_width_ = 720;
        uint32_t stream_height_ = 1440;

        cxrFramesLatched framesLatched_ = {};
        bool latched_ = false;

        std::mutex state_mutex_;
        cxrMatrix34 pose_matrix_[kQueueLen] = {};
        cxrDeviceDesc device_desc_ = {};
        int current_idx_ = 0;

        int fps_ = 60;

        std::shared_ptr<oboe::AudioStream> recording_stream_{};
        std::shared_ptr<oboe::AudioStream> playback_stream_{};

        cxrConnectionStats stats_ = {};
        int frames_until_stats_ = 60;
    };

// need to decl our static variable.
    bool HelloArApplication::exiting_ = false;

    HelloArApplication::HelloArApplication(AAssetManager *asset_manager)
            : asset_manager_(asset_manager) {
        cloudxr_client_ = std::make_unique<HelloArApplication::CloudXRClient>();
        exiting_ = false; // reset static here in case library remains resident..
    }

    HelloArApplication::~HelloArApplication() {

    }

// use for any deeper, failure-possible init of app, or cxr client.
    bool HelloArApplication::Init() {
        return cloudxr_client_->Init();
    }

// pass server address direct to client.
    void HelloArApplication::HandleLaunchOptions(std::string &cmdline) {
        cloudxr_client_->HandleLaunchOptions(cmdline);
    }

// pass command line args direct to client.
    void HelloArApplication::SetArgs(const std::string &args) {
        cloudxr_client_->SetArgs(args);
    }

// pass server address direct to client.
    std::string HelloArApplication::GetServerIp() {
        return cloudxr_client_->GetServerAddr();
    }

    void HelloArApplication::OnPause() {
        LOGI("OnPause()");
        cloudxr_client_->Teardown();
    }

    void HelloArApplication::OnResume(void *env, int fps) {
        cloudxr_client_->SetFps(fps);
    }

    void HelloArApplication::OnDisplayGeometryChanged(int display_rotation,
                                                      int width, int height) {
        LOGI("OnDisplayGeometryChanged(%d, %d, %d)", display_rotation, width, height);
        display_rotation_ = display_rotation;
        display_width_ = width;
        display_height_ = height;
        cloudxr_client_->SetStreamRes(display_width_, display_height_, display_rotation);
    }

    int HelloArApplication::TryCxrConnect(glm::mat4 projection_mat) {
        if (!cloudxr_client_->IsRunning()) {
            cloudxr_client_->SetProjectionMatrix(projection_mat);
            cxrError status = cloudxr_client_->Connect();
            // for sync connection, this will do for now to error check..
            if (status != cxrError_Success) {
                exiting_ = true;
                return -1; // TODO: real error codes?
            }
        }
        return 0;
    }

    void HelloArApplication::TryCxrRelease() {
        if (cloudxr_client_->Latch() == cxrError_Success) {
            cloudxr_client_->Release();
        }
    }

    void HelloArApplication::CxrUpdateLightProps(float direction[], float intensity[],
                                                 float ambient_spherical_harmonics[]) {
        cloudxr_client_->UpdateLightProps(direction, intensity, ambient_spherical_harmonics);
    }

    int HelloArApplication::GetCxrOffset() {
        return cloudxr_client_->DetermineOffset();
    }

    int HelloArApplication::GetCxrError() {
        cxrError error = cloudxr_client_->Latch();
        if (error == cxrError_Receiver_Not_Running) {
            exiting_ = true;
        }
        return error;
    }

    void HelloArApplication::SetCxrPoseMat(glm::mat4 matrix) {
        // Setup pose matrix with our base frame
        const glm::mat4 cloudxr_pose_mat = base_frame_ * glm::inverse(matrix);
        cloudxr_client_->SetPoseMatrix(cloudxr_pose_mat);
    }

    void HelloArApplication::UpdateFrame(glm::mat4 matrix) {
        base_frame_ = glm::inverse(matrix);
        base_frame_calibrated_ = true;
    }

    bool HelloArApplication::IsFrameCalibrated() {
        return base_frame_calibrated_;
    }

    bool HelloArApplication::IsCxrRunning() {
        return cloudxr_client_->IsRunning();
    }

    bool HelloArApplication::IsCxrUsingEnvLighting() {
        return cloudxr_client_->GetUseEnvLighting();
    }

// Render the scene.
// return value 0 means that Java should finish and clean up.
    void HelloArApplication::OnDrawFrame(float color_correction[4]) {
        // Composite CloudXR frame to the screen
        cloudxr_client_->Render(color_correction);
        cloudxr_client_->Release();
        cloudxr_client_->Stats();
    }

    void HelloArApplication::OnTouched(float x, float y, bool longPress) {
        // if base frame is calibrated and user is not asking to reset, pass touches to server
        if (base_frame_calibrated_ && !longPress) {
            if (cloudxr_client_->IsRunning())
                cloudxr_client_->HandleTouch(x, y);
            return;  // TODO: should return true/false for handled/used the event...
        }

        // Reset calibration on a long press
        if (longPress) {
            base_frame_calibrated_ = false;
            return;
        }
    }
}  // namespace hello_ar
