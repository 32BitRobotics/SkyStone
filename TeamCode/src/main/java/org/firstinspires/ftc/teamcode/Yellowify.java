package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public final class Yellowify {
    private Yellowify() { }

    private static double threshold = -1;

    public static void process(Mat input, Mat mask) {
        List<Mat> channels = new ArrayList<>();

        Mat lab = new Mat(input.size(), 0);
        Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);
        Mat temp = new Mat();
        Core.inRange(input, new Scalar(0,0,0), new Scalar(255,255,164), temp);
        Mat mask2 = new Mat(input.size(), 0);
        temp.copyTo(mask2);
        input.copyTo(input, mask2);
        mask2.release();
        temp.release();
        lab.release();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(input,input,new Size(3,3),0);
        Core.split(input, channels);
        if(channels.size() > 0){
            Imgproc.threshold(channels.get(1), mask, threshold, 255, Imgproc.THRESH_BINARY_INV);
        }
    }
}
