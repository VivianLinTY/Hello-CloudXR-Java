package com.google.ar.core.examples.java.helloar;

public class Log {
    private static final String APP_TAG = "HelloCloudXr";
    private static final boolean DEBUG = true;

    public static void v(String tag, String message) {
        if (DEBUG) {
            android.util.Log.v(APP_TAG + " > " + tag, message);
        }
    }

    public static void i(String tag, String message) {
        if (DEBUG) {
            android.util.Log.i(APP_TAG + " > " + tag, message);
        }
    }

    public static void d(String tag, String message) {
        if (DEBUG) {
            android.util.Log.d(APP_TAG + " > " + tag, message);
        }
    }

    public static void w(String tag, String message) {
        android.util.Log.w(APP_TAG + " > " + tag, message);
    }

    public static void w(String tag, String message, Exception e) {
        android.util.Log.w(APP_TAG + " > " + tag, message, e);
    }

    public static void e(String tag, String message) {
        android.util.Log.e(APP_TAG + " > " + tag, message);
    }

    public static void e(String tag, String message, Exception e) {
        android.util.Log.e(APP_TAG + " > " + tag, message, e);
    }

    public static void e(String tag, String message, Throwable t) {
        android.util.Log.e(APP_TAG + " > " + tag, message, t);
    }
}
