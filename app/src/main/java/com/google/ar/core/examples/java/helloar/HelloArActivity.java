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

package com.google.ar.core.examples.java.helloar;

import android.content.Context;
import android.content.SharedPreferences;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Size;
import android.view.MotionEvent;
import android.widget.Toast;
import com.google.ar.core.Anchor;
import com.google.ar.core.ArCoreApk;
import com.google.ar.core.Camera;
import com.google.ar.core.CameraConfig;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.HitResult;
import com.google.ar.core.Plane;
import com.google.ar.core.Point;
import com.google.ar.core.Point.OrientationMode;
import com.google.ar.core.Session;
import com.google.ar.core.Trackable;
import com.google.ar.core.TrackingState;
import com.google.ar.core.examples.java.common.helpers.CameraPermissionHelper;
import com.google.ar.core.examples.java.common.helpers.DisplayRotationHelper;
import com.google.ar.core.examples.java.common.helpers.FullScreenHelper;
import com.google.ar.core.examples.java.common.helpers.SnackbarHelper;
import com.google.ar.core.examples.java.common.helpers.TapHelper;
import com.google.ar.core.examples.java.common.helpers.TrackingStateHelper;
import com.google.ar.core.examples.java.common.rendering.BackgroundRenderer;
import com.google.ar.core.examples.java.common.rendering.PlaneRenderer;
import com.google.ar.core.exceptions.CameraNotAvailableException;
import com.google.ar.core.exceptions.UnavailableApkTooOldException;
import com.google.ar.core.exceptions.UnavailableArcoreNotInstalledException;
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException;
import com.google.ar.core.exceptions.UnavailableSdkTooOldException;
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException;

import java.io.IOException;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * This is a simple example that shows how to create an augmented reality (AR) application using the
 * ARCore API. The application will display any detected planes and will allow the user to tap on a
 * plane to place a 3d model of the Android robot.
 */
public class HelloArActivity extends AppCompatActivity implements GLSurfaceView.Renderer {
  private static final String TAG = HelloArActivity.class.getSimpleName();

  private final String ipAddrPref = "cxr_last_server_ip_addr";
  private final String cloudAnchorPref = "cxr_last_cloud_anchor";

  private SharedPreferences prefs = null;
  private boolean wasResumed = false;
  private boolean exiting = false;
  private String cmdlineFromIntent = "";
  private int cam_image_width = 1920;
  private int cam_image_height = 1080;

  // Opaque native pointer to the native application instance.
  private long nativeApplication;

  // Rendering. The Renderers are created here, and initialized when the GL surface is created.
  private GLSurfaceView surfaceView;

  private boolean installRequested;

  private Session session;
  private final SnackbarHelper messageSnackbarHelper = new SnackbarHelper();
  private DisplayRotationHelper displayRotationHelper;
  private final TrackingStateHelper trackingStateHelper = new TrackingStateHelper(this);
  private TapHelper tapHelper;

  private final BackgroundRenderer backgroundRenderer = new BackgroundRenderer();
  private final PlaneRenderer planeRenderer = new PlaneRenderer();

  // Temporary matrix allocated here to reduce number of allocations for each frame.
  private final float[] anchorMatrix = new float[16];
  private static final float[] DEFAULT_COLOR = new float[] {0f, 0f, 0f, 0f};

  private static final String SEARCHING_PLANE_MESSAGE = "Searching for surfaces...";

  private Anchor anchor;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    prefs = getSharedPreferences("cloud_xr_prefs", Context.MODE_PRIVATE);

    setContentView(R.layout.activity_main);
    surfaceView = findViewById(R.id.surfaceview);
    displayRotationHelper = new DisplayRotationHelper(/*context=*/ this);

    // Set up tap listener.
    tapHelper = new TapHelper(/*context=*/ this);
    surfaceView.setOnTouchListener(tapHelper);

    // Set up renderer.
    surfaceView.setPreserveEGLContextOnPause(true);
    surfaceView.setEGLContextClientVersion(2);
    surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0); // Alpha used for plane blending.
    surfaceView.setRenderer(this);
    surfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
    surfaceView.setWillNotDraw(false);

    installRequested = false;

    // check for any data passed to our activity that we want to handle
    cmdlineFromIntent = getIntent().getStringExtra("args");

    nativeApplication = JniInterface.createNativeApplication(getAssets());
  }

  @Override
  protected void onResume() {
    super.onResume();

    // We require camera, internet, and file permissions to function.
    // If we don't yet have permissions, need to go ask the user now.
    if (!PermissionHelper.hasPermissions(this)) {
      PermissionHelper.requestPermissions(this);
      Log.v(TAG, "early return, waiting on permission callback");
      return;
    }

    // if we had permissions, we can move on to checking launch options.
    checkLaunchOptions();
  }

  protected void checkLaunchOptions() {
    if (wasResumed || ServerIPDialog.isShowing())
      return;

    Log.v(TAG, "Checking launch options..");

    // we're done with permission checks, so can tell native now is safe to
    // try to load files and such.
    JniInterface.handleLaunchOptions(nativeApplication, cmdlineFromIntent);

    // check if the native code already has a server IP, and if so
    // we will skip presenting the IP entry dialog for now...
    String jniIpAddr = JniInterface.getServerIp(nativeApplication);
    if (jniIpAddr.isEmpty()) {
      String prevIP = prefs.getString(ipAddrPref, "");
      String prevCloudAnchor = prefs.getString(cloudAnchorPref, "");
      ServerIPDialog.show(this, prevIP, prevCloudAnchor);
    } else {
      doResume();
    }
  }

  public void doResume() {
    if (session == null) {
      Exception exception = null;
      String message = null;
      try {
        switch (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
          case INSTALL_REQUESTED:
            installRequested = true;
            return;
          case INSTALLED:
            break;
        }

        // ARCore requires camera permissions to operate. If we did not yet obtain runtime
        // permission on Android M and above, now is a good time to ask the user for it.
        if (!CameraPermissionHelper.hasCameraPermission(this)) {
          CameraPermissionHelper.requestCameraPermission(this);
          return;
        }

        // Create the session.
        session = new Session(/* context= */ this);

      } catch (UnavailableArcoreNotInstalledException
              | UnavailableUserDeclinedInstallationException e) {
        message = "Please install ARCore";
        exception = e;
      } catch (UnavailableApkTooOldException e) {
        message = "Please update ARCore";
        exception = e;
      } catch (UnavailableSdkTooOldException e) {
        message = "Please update this app";
        exception = e;
      } catch (UnavailableDeviceNotCompatibleException e) {
        message = "This device does not support AR";
        exception = e;
      } catch (Exception e) {
        message = "Failed to create AR session";
        exception = e;
      }

      if (message != null) {
        messageSnackbarHelper.showError(this, message);
        Log.e(TAG, "Exception creating session", exception);
        return;
      }
    }

    // Note that order matters - see the note in onPause(), the reverse applies here.
    try {
      session.resume();
    } catch (CameraNotAvailableException e) {
      messageSnackbarHelper.showError(this, "Camera not available. Try restarting the app.");
      session = null;
      return;
    }

    surfaceView.onResume();
    displayRotationHelper.onResume();

    CameraConfig cameraConfig = session.getCameraConfig();
    int fps = cameraConfig.getFpsRange().getUpper();
    if (fps >= 60) {
      fps = 60;
    }
    JniInterface.onResume(nativeApplication, fps);
    Size size = cameraConfig.getImageSize();
    cam_image_width = size.getWidth();
    cam_image_height = size.getHeight();
    Log.d("Zoe","cam_image_width="+cam_image_width+", texture_width="+cameraConfig.getTextureSize().getWidth());

    if (JniInterface.isCxrUsingEnvLighting(nativeApplication)) {
      Config config = session.getConfig();
      config.setLightEstimationMode(Config.LightEstimationMode.ENVIRONMENTAL_HDR);
      session.configure(config);
    }
    wasResumed = true;
  }

  @Override
  public void onPause() {
    super.onPause();
    if (wasResumed) {
      JniInterface.onPause(nativeApplication);
      if (session != null) {
        // Note that the order matters - GLSurfaceView is paused first so that it does not try
        // to query the session. If Session is paused before GLSurfaceView, GLSurfaceView may
        // still call session.update() and get a SessionPausedException.
        displayRotationHelper.onPause();
        surfaceView.onPause();
        session.pause();
      }
      wasResumed = false;
    }
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
    if (!CameraPermissionHelper.hasCameraPermission(this)) {
      Toast.makeText(this, "Camera permission is needed to run this application", Toast.LENGTH_LONG)
          .show();
      if (!CameraPermissionHelper.shouldShowRequestPermissionRationale(this)) {
        // Permission denied with checking "Do not ask again".
        CameraPermissionHelper.launchPermissionSettings(this);
      }
      finish();
    }
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus) {
    super.onWindowFocusChanged(hasFocus);
    FullScreenHelper.setFullScreenOnWindowFocusChanged(this, hasFocus);
  }

  @Override
  public void onSurfaceCreated(GL10 gl, EGLConfig config) {
    GLES20.glClearColor(exiting ? 0.0f : 0.3f, 0.0f, 0.0f, 1.0f);

    // Prepare the rendering objects. This involves reading shaders, so may throw an IOException.
    try {
      // Create the texture and pass it to ARCore session to be filled during update().
      backgroundRenderer.createOnGlThread(/*context=*/ this, cam_image_width, cam_image_height);
      planeRenderer.createOnGlThread(/*context=*/ this, "models/trigrid.png");
    } catch (Exception e) {
      Log.e(TAG, "Failed to read an asset file", e);
    }
  }

  @Override
  public void onSurfaceChanged(GL10 gl, int width, int height) {
    displayRotationHelper.onSurfaceChanged(width, height);
    GLES20.glViewport(0, 0, width, height);
  }

  @Override
  public void onDestroy() {
    super.onDestroy();
    // Synchronized to avoid racing onDrawFrame.
    synchronized (this) {
      JniInterface.destroyNativeApplication(nativeApplication);
      nativeApplication = 0;
      wasResumed = false;
    }
  }

  @Override
  public void onDrawFrame(GL10 gl) {
    // Clear screen to notify driver it should not load any pixels from previous frame.
    GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

    if (exiting) {
      return;
    }

    if (session == null) {
      return;
    }
    // Notify ARCore session that the view size changed so that the perspective matrix and
    // the video background can be properly adjusted.
    displayRotationHelper.updateSessionIfNeeded(nativeApplication, session);

    try {
      session.setCameraTextureName(backgroundRenderer.getTextureId());

      // Obtain the current frame from ARSession. When the configuration is set to
      // UpdateMode.BLOCKING (it is by default), this will throttle the rendering to the
      // camera framerate.
      Frame frame = session.update();
      Camera camera = frame.getCamera();

      // Handle one tap per frame.
      handleTap(frame, camera);

      backgroundRenderer.draw(frame, -1);
      GLES20.glViewport(0, 0, displayRotationHelper.getDisplaySize().getWidth(),
              displayRotationHelper.getDisplaySize().getHeight());
      if (!JniInterface.isCxrRunning(nativeApplication) || !JniInterface.isFrameCalibrated(nativeApplication)) {
        // If frame is ready, render camera preview image to the GL surface.
        backgroundRenderer.draw(frame, 0);
      }

      if (camera.getTrackingState() != TrackingState.TRACKING) {
        return;
      }

      // Keep the screen unlocked while tracking, but allow it to lock when tracking stops.
      trackingStateHelper.updateKeepScreenOnFlag(camera.getTrackingState());

      // If not tracking, don't draw 3D objects, show tracking failure reason instead.
      if (camera.getTrackingState() == TrackingState.PAUSED) {
        messageSnackbarHelper.showMessage(
            this, TrackingStateHelper.getTrackingFailureReasonString(camera));
        return;
      }

      // We need to (re)calibrate but CloudXR client is running - continue
      // pulling the frames. There'll be a lag otherwise.
      if (!JniInterface.isFrameCalibrated(nativeApplication) && JniInterface.isCxrRunning(nativeApplication)) {
        JniInterface.tryCxrRelease(nativeApplication);
      }

      // Get projection matrix.
      float[] projmtx = new float[16];
      camera.getProjectionMatrix(projmtx, 0, 0.1f, 100.0f);

      if (JniInterface.isFrameCalibrated(nativeApplication)) {
        // Get camera matrix and draw.
        float[] viewmtx = new float[16];
        camera.getViewMatrix(viewmtx, 0);

        if (null != anchor) {
          if (anchor.getTrackingState() == TrackingState.TRACKING) {
            anchor.getPose().toMatrix(anchorMatrix, 0);
            JniInterface.updateFrame(nativeApplication, anchorMatrix);
          }
        }

        if (!JniInterface.isCxrRunning(nativeApplication)) {
          int cxrStatus = JniInterface.tryCxrConnect(nativeApplication, projmtx);
          if (-1 == cxrStatus) {
            exiting = true;
            return;
          }
        }

        int status = JniInterface.getCxrError(nativeApplication);
        if (status != 0) {
          if (status == 7) {
            exiting = true;
            return;
          }
          // else
          // TODO: code should handle other potential errors that are non-fatal, but
          //  may be enough to need to disconnect or reset view or other interruption cases.
        }

        boolean hasFrame = 0 == status;
        int pose_offset = hasFrame ? JniInterface.getCxrOffset(nativeApplication) : 0;
        GLES20.glViewport(0, 0, displayRotationHelper.getDisplaySize().getWidth(),
                displayRotationHelper.getDisplaySize().getHeight());
        backgroundRenderer.draw(frame, pose_offset);
        JniInterface.setCxrPoseMat(nativeApplication, viewmtx);

        // Compute lighting from average intensity of the image.
        // The first three components are color scaling factors.
        // The last one is the average pixel intensity in gamma space.
        final float[] colorCorrectionRgba = new float[]{1.f, 1.f, 1.f, 0.466f};
        if (JniInterface.isCxrUsingEnvLighting(nativeApplication)) {
          float[] direction = frame.getLightEstimate().getEnvironmentalHdrMainLightDirection();
          float[] intensity = frame.getLightEstimate().getEnvironmentalHdrMainLightIntensity();
          float[] ambient_spherical_harmonics = frame.getLightEstimate().getEnvironmentalHdrAmbientSphericalHarmonics();
          JniInterface.cxrUpdateLightProps(nativeApplication, direction, intensity, ambient_spherical_harmonics);
        } else {
          frame.getLightEstimate().getColorCorrection(colorCorrectionRgba, 0);
        }
        if (hasFrame) {
          GLES20.glViewport(0, 0, displayRotationHelper.getDisplaySize().getWidth(),
                  displayRotationHelper.getDisplaySize().getHeight());
          JniInterface.onGlSurfaceDrawFrame(nativeApplication, colorCorrectionRgba);
        }
        return;
      }

      // No tracking error at this point. If we detected any plane, then hide the
      // message UI, otherwise show searchingPlane message.
      if (hasTrackingPlane()) {
        messageSnackbarHelper.hide(this);
      } else {
        messageSnackbarHelper.showMessage(this, SEARCHING_PLANE_MESSAGE);
      }

      // Visualize planes.
      planeRenderer.drawPlanes(
          session.getAllTrackables(Plane.class), camera.getDisplayOrientedPose(), projmtx);

      // Visualize anchors created by touch.
      if (null != anchor) {
        if (anchor.getTrackingState() != TrackingState.TRACKING) {
          return;
        }
        // Get the current pose of an Anchor in world space. The Anchor pose is updated
        // during calls to session.update() as ARCore refines its estimate of the world.
        anchor.getPose().toMatrix(anchorMatrix, 0);

        // Update and draw the model and its shadow.
        JniInterface.updateFrame(nativeApplication, anchorMatrix);
      }

    } catch (Throwable t) {
      // Avoid crashing the application due to unhandled exceptions.
      Log.e(TAG, "Exception on the OpenGL thread",t);
    }
  }

  // Handle only one tap per frame, as taps are usually low frequency compared to frame rate.
  private void handleTap(Frame frame, Camera camera) {
    TapHelper.TapEvent tap = tapHelper.poll();
    if (tap != null && camera.getTrackingState() == TrackingState.TRACKING) {
      for (HitResult hit : frame.hitTest(tap.event)) {
        // Check if any plane was hit, and if it was hit inside the plane polygon
        Trackable trackable = hit.getTrackable();
        // Creates an anchor if a plane or an oriented point was hit.
        if ((trackable instanceof Plane
                && ((Plane) trackable).isPoseInPolygon(hit.getHitPose())
                && (PlaneRenderer.calculateDistanceToPlane(hit.getHitPose(), camera.getPose()) > 0))
            || (trackable instanceof Point
                && ((Point) trackable).getOrientationMode()
                    == OrientationMode.ESTIMATED_SURFACE_NORMAL)) {
          // Assign a color to the object for rendering based on the trackable type
          // this anchor attached to. For AR_TRACKABLE_POINT, it's blue color, and
          // for AR_TRACKABLE_PLANE, it's green color.
          float[] objColor;
          if (trackable instanceof Point) {
            objColor = new float[] {66.0f, 133.0f, 244.0f, 255.0f};
          } else if (trackable instanceof Plane) {
            objColor = new float[] {139.0f, 195.0f, 74.0f, 255.0f};
          } else {
            objColor = DEFAULT_COLOR;
          }

          // Adding an Anchor tells ARCore that it should track this position in
          // space. This anchor is created on the Plane to place the 3D model
          // in the correct position relative both to the world and to the plane.
          anchor = hit.createAnchor();
          break;
        }
      }
    }
    if (null != tap) {
      JniInterface.onTouched(nativeApplication, tap.event.getX(), tap.event.getY(), tap.isLongPress);
      if (tap.isLongPress && null != anchor) {
        anchor.detach();
        anchor = null;
      }
    }
  }

  /** Checks if we detected at least one plane. */
  private boolean hasTrackingPlane() {
    for (Plane plane : session.getAllTrackables(Plane.class)) {
      if (plane.getTrackingState() == TrackingState.TRACKING) {
        return true;
      }
    }
    return false;
  }

  public void setParams(String ip, String cloudAnchorId, boolean hostCloudAnchor) {
    SharedPreferences.Editor prefedit = prefs.edit();
    prefedit.putString(ipAddrPref, ip);
    prefedit.putString(cloudAnchorPref, cloudAnchorId);
    prefedit.commit();

    JniInterface.setArgs(nativeApplication, "-s " + ip + " -c " +
            (hostCloudAnchor ? "host" : cloudAnchorId));
  }
}
