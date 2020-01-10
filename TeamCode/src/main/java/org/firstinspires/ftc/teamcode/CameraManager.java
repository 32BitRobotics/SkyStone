package org.firstinspires.ftc.teamcode;

import android.bluetooth.le.ScanRecord;
import android.content.Context;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public final class CameraManager {
    OpenCvCamera camera;
    CMPipeline pipeline;
    boolean[] thresh_crosses = new boolean[10];
    public boolean hasExec = false;

    public List<MatOfPoint> contourList;

    enum Stage
    {
        YCbCr_CHAN2,
        THRESHOLD,
        contourList_OVERLAYED_ON_FRAME,
        COM,
        RAW_IMAGE,
    }

    public CameraManager(Context appContext) {
        int cameraMonitorViewId = appContext.getResources().getIdentifier("cameraMonitorViewId", "id", appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();
        pipeline = new CMPipeline(this);
        camera.setPipeline(pipeline);
        camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
    }

    public final class CMPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat contourListOnFrameMat = new Mat();
        Mat stuffMat = new Mat();
        Mat rectanglesOnMat = new Mat();
        Mat resultMat = new Mat();
        public List<MatOfPoint> contourListList = new ArrayList<>();

        Rect[][] rects = new Rect[][] {
                new Rect[] {
                        new Rect(0,0,240,640),
                        new Rect(240,0,240,640)
                },
                new Rect[] {
                        new Rect(0,0,160, 640),
                        new Rect(160, 0, 160, 640),
                        new Rect(320, 0, 160, 640)
                }
        };
        int curRect = 0;

        double threshold = 24000.0;

        boolean[] thresh_crosses = new boolean[10];

        int numcontourListFound;

        private CameraManager cm;

        private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
        private Stage[] stages = Stage.values();

        public CMPipeline(CameraManager cm) {
            this.cm = cm;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
*/

            /*int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];*/

            // advance rect on tap
            //curRect++;
            //if (curRect > rects.length - 1) {
            //    curRect = 0;
            //}

            // advance threshold on tap
            threshold += 2000.0;
        }

        double findYellowArea(Mat subarea) {
            Imgproc.cvtColor(subarea, subarea, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(subarea, subarea, 2);
            Imgproc.threshold(subarea, subarea, 102, 225, Imgproc.THRESH_BINARY_INV);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(subarea, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            double area = 0.0;
            for (MatOfPoint pt : contours) {
                area += Imgproc.contourArea(pt);
            }
            return area;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contourListList.clear();

            input.copyTo(resultMat);
            Scalar red = new Scalar(255, 0, 0);
            Scalar green = new Scalar(0, 255, 0);
            int i = 0;
            for (Rect rect : rects[curRect]) {
                double area = findYellowArea(input.submat(rect));
                Imgproc.rectangle(resultMat, rect, area > threshold ? green : red, 10);
                thresh_crosses[i++] = area < threshold;
            }

            cm.thresh_crosses = thresh_crosses;
            cm.hasExec = true;

            return resultMat;

            /*
             * This pipeline finds the contourList of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
            /*Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 100, 255, Imgproc.THRESH_BINARY);

            /*Imgproc.findContours(thresholdMat, contourListList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            numcontourListFound = contourListList.size();

            // center of mass
            float x = 0.0f;
            float y = 0.0f;
            int size = 0;

            for (MatOfPoint pt : contourListList) {
                for (Point pt1 : pt.toArray()) {
                    x += pt1.x;
                    y += pt1.y;
                    size += 1;
                }
            }

            Point pt = new Point(x/size, y/size);


            input.copyTo(contourListOnFrameMat);
            Imgproc.drawContours(contourListOnFrameMat, contourListList, -1, new Scalar(0, 0, 255), 3, 8);
            contourListOnFrameMat = contourListOnFrameMat.submat(new Rect(240,0,240,320));
            inputnd center of ma.copyTo(stuffMat);
            Imgproc.circle(stuffMat, pt, 10, new Scalar(255, 0, 0));*/

            //cm.contourList = contourListList;
/*
            switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN2:
                {
                    return yCbCrChan2Mat;
                }

                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case COM: {
                    return stuffMat;
                }

                case contourList_OVERLAYED_ON_FRAME:
                {
                    return contourListOnFrameMat;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }*/
        }

        public int getNumcontourListFound()
        {
            return numcontourListFound;
        }
    }
}
