#Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
#Permission is hereby granted, free of charge, to any person obtaining a
#copy of this software and associated documentation files (the "Software"),
#to deal in the Software without restriction, including without limitation
#the rights to use, copy, modify, merge, publish, distribute, sublicense,
#and/or sell copies of the Software, and to permit persons to whom the
#Software is furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
#THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#DEALINGS IN THE SOFTWARE.

# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

# Import the glm header file from the NDK.
add_library( glm INTERFACE )
set_target_properties( glm PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${GLM_INCLUDE})

# Import Oboe
add_library(oboe-lib SHARED IMPORTED)
set_target_properties(oboe-lib PROPERTIES IMPORTED_LOCATION
        ${OBOE_LIBPATH}/${ANDROID_ABI}/liboboe.so
        )
include_directories(${OBOE_INCLUDE})

# Import CloudXR
add_library(cloudxr-lib SHARED IMPORTED)
set_target_properties(cloudxr-lib PROPERTIES IMPORTED_LOCATION
        ${CLOUDXR_LIBPATH}/${ANDROID_ABI}/libCloudXRClient.so
        )
include_directories(${CLOUDXR_INCLUDE})

# Include shared headers
include_directories(${C_SHARED_INCLUDE})

# This is the main app library.
add_library(hello_cloudxr_native SHARED
        src/main/cpp/hello_ar_application.cc
        src/main/cpp/jni_interface.cc
        src/main/cpp/util.cc)

target_include_directories(hello_cloudxr_native PRIVATE
        src/main/cpp)
target_link_libraries(hello_cloudxr_native
        cloudxr-lib
        oboe-lib
        android
        log
        glm
        EGL)
