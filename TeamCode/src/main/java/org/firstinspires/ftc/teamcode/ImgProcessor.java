package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamClient;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;
import java.util.concurrent.FutureTask;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public final class ImgProcessor {

    public CameraStreamClient csc;
    public boolean[] current = new boolean[4];
    public ImgProcessor(Context appContext) {
        csc = CameraStreamClient.getInstance();
        csc.setListener(new CameraStreamClient.Listener() {
            @Override
            public void onStreamAvailableChange(boolean available) {

            }

            @Override
            public void onFrameBitmap(Bitmap frameBitmap) {
              // TOOO: process
              current[0] = false;
              current[1] = true;
              current[2] = false;
              current[3] = true;
            }
        });
    }

    public boolean[] processImage() {
      return current;
    }
}
