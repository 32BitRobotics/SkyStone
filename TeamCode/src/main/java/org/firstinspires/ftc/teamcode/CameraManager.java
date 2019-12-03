package org.firstinspires.ftc.teamcode;

import android.content.Context;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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

    public List<MatOfPoint> contourList;

    enum Stage
    {
        YCbCr_CHAN2,
        THRESHOLD,
        contourList_OVERLAYED_ON_FRAME,
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
        public List<MatOfPoint> contourListList = new ArrayList<>();
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

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contourListList.clear();

            /*
             * This pipeline finds the contourList of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.findContours(thresholdMat, contourListList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numcontourListFound = contourListList.size();
            input.copyTo(contourListOnFrameMat);
            Imgproc.drawContours(contourListOnFrameMat, contourListList, -1, new Scalar(0, 0, 255), 3, 8);

            cm.contourList = contourList;

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
            }
        }

        public int getNumcontourListFound()
        {
            return numcontourListFound;
        }
    }
}
