package org.firstinspires.ftc.teamcode.vision;


import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Disabled
public class BlueDetectionLeft implements VisionProcessor {
    Telemetry telemetry;
    Mat mat = new Mat();

    Scalar lowBlueHSV = new Scalar(90, 50, 50);
    Scalar highBlueHSV = new Scalar(130, 255, 255);

//    Scalar lower_red1 = new Scalar(0,50,50);
//    Scalar upper_red1 = new Scalar(10,255,255);
//    Scalar lower_red2 = new Scalar(160,50,50);
//    Scalar upper_red2 = new Scalar(180,255,255);

    static int readout = 0;

    public static double leftValue;
    public static double rightValue;
    public static double centerValue;
    public static double leftPixels;
    public static double rightPixels;
    public static double centerPixels;

    public static double thresh = 0.06;


    static final Rect LEFT_ROI = new Rect(
            new Point(0, 120),
            new Point(100, 320));
    static final Rect CENTER_ROI = new Rect(
            new Point(140,150),
            new Point(140+300,150+150));
    static final Rect RIGHT_ROI = new Rect(
            new Point(500, 120),
            new Point(640, 320));

    Scalar No = new Scalar(255, 0, 0);
    Scalar Yes = new Scalar(0, 255, 0);


    public BlueDetectionLeft(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }



    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.blur(frame,frame, new Size(4,4));

        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);


        Core.inRange(mat, lowBlueHSV, highBlueHSV, mat);
//        Core.inRange(mat, lower_red1,upper_red1,mat);
//        Core.inRange(mat, lower_red2,upper_red2,matExtra);

//        Core.bitwise_or(mat,matExtra,mat);


        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        leftPixels = Core.countNonZero(left);
        rightPixels = Core.countNonZero(right);
        centerPixels = Core.countNonZero(center);

        leftValue = leftPixels / LEFT_ROI.area();
        centerValue = centerPixels / CENTER_ROI.area();
        rightValue = rightPixels / RIGHT_ROI.area();

        left.release();
        center.release();
        right.release();

        if (leftPixels>rightPixels && leftPixels>centerPixels&&(leftValue>thresh)){
            readout = 1;
        }else if (centerPixels>rightPixels && centerPixels>leftPixels&&(centerValue>thresh)){
            readout = 2;
        }else if (rightPixels>leftPixels && rightPixels>centerPixels&&(rightValue>thresh)){
            readout = 3;
        }else{
            readout = 0;
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(mat, LEFT_ROI, readout == 1? Yes:No);
        Imgproc.rectangle(mat, CENTER_ROI, readout == 2? Yes:No);
        Imgproc.rectangle(mat, RIGHT_ROI, readout == 3? Yes:No);

        mat.copyTo(frame);
        mat.release();
        return null;
    }

    public static int getReadout(){
        return readout;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }

}