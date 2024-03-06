package org.firstinspires.ftc.teamcode.auto;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class TestAuto extends LinearOpMode {

    // PID constants for turning
    public static double turnKp = 0.6; //tuned 1/31
    public static double turnKi = 0.025; //tuned 1/31
    public static double turnKd = 0.0175; //tuned 1/31
    // PID constants for forward/back
    public static double Kp_dist = 0.1; //tuned 2/1
    public static double Ki_dist = 0.006; //tuned 2/1
    public static double Kd_dist = 0.2; //tuned 2/1
    // PID constants for heading
    public static double Kp_heading = 0.07; //tuned 1/30
    public static double Ki_heading = 0.015; //tuned 1/30
    public static double Kd_heading = 0.1; //tuned 1/30
    // PID constants for strafing
    public static double Kp_strafe = 0.225; //tuned 1/30
    public static double Ki_strafe = 0.05; //tuned 1/30
    public static double Kd_strafe = 0.125; //tuned 1/30

    // Hardware Encoder Scale.
    public static double encoder_tick_scale = 40; //tuned 1/30
    public static double heading_correct_scale = 6; //tuned 1/30
    public static double ticks_per_inch = 1865.253;
    public static double PIDOUTPUTSCALE = 60; //tuned 1/29

    // hardware variables
    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightBack = null;
    public BHI260IMU imu = null;

    // don't touch (ㆆ_ㆆ)
    private double turn_previousError = 0;      //don't touch (ㆆ_ㆆ)
    private double turn_integral = 0;           //don't touch (ㆆ_ㆆ)
    private double previousErrorStrafe = 0;     //don't touch (ㆆ_ㆆ)
    private double integralStrafe = 0;          //don't touch (ㆆ_ㆆ)
    private double previousErrorDist = 0;       //don't touch (ㆆ_ㆆ)
    private double integralDist = 0;            //don't touch (ㆆ_ㆆ)
    private double previousErrorHeading = 0;    //don't touch (ㆆ_ㆆ)
    private double integralHeading = 0;         //don't touch (ㆆ_ㆆ)

    //
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public boolean targetFound = false;    // Set to true when an AprilTag target is detected

    // 359 AUTON!!!! ೭੧(❛〜❛✿)੭೨
    @Override
    public void runOpMode() throws InterruptedException {

        // init phase
        while (opModeInInit()) {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            imu = hardwareMap.get(BHI260IMU.class, "imu");
            imu.resetYaw();

            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            initAprilTag();
            //if (USE_WEBCAM)
            //    setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }

        // run auton here!! ٩(˘◡˘ )
        if (isStarted()) {
            turn_test();
        }
    }

    // auton routes

    public void turn_test(){
        robot_turn(90);
        sleep(250);
        robot_turn(-90);
        sleep(250);
        robot_turn(-90);
        sleep(250);
        robot_turn(90);
        sleep(250);
        robot_turn(45);
        sleep(250);
        robot_turn(-45);
        sleep(250);
        robot_turn(-45);
        sleep(250);
        robot_turn(45);
        sleep(250);
    }
    public void far_red_middle(){
        robot_move(-30, 0.5);
        sleep(150);
        robot_move(6, 0.5);
        sleep(150);
        robot_strafe(-12, 0.5);
        sleep(150);
        robot_move(-30, 0.5);
        sleep(150);
        robot_turn(-90);
        sleep(150);
        robot_move(-96, 0.5);
    }
    public void far_red_right(){
        robot_move(-20, 0.5);
        sleep(150);
        robot_turn(-60);
        sleep(150);
        robot_move(-11, 0.5);
        sleep(150);
        robot_move(11, 0.5);
        sleep(150);
        robot_turn(60);
        sleep(150);
        robot_move(-30, 0.5);
        sleep(150);
        robot_turn(-87);
        sleep(150);
        robot_move(-84, 0.5);
    }
    public void far_red_left(){
        robot_move(-20, 0.5);
        sleep(150);
        robot_turn(45);
        sleep(150);
        robot_move(-12, 0.5);
        sleep(150);
        robot_move(13, 0.5);
        sleep(150);
        robot_turn(-45);
        sleep(150);
        robot_move(-34, 0.5);
        sleep(150);
        robot_turn(-86);
        sleep(150);
        robot_move(-86, 0.5);
    }
    public void far_blue_middle(){
        robot_move(-30, 0.5);
        sleep(150);
        robot_move(6, 0.5);
        sleep(150);
        robot_strafe(12, 0.5);
        sleep(150);
        robot_move(-30, 0.5);
        sleep(150);
        robot_turn(92);
        sleep(150);
        robot_move(-96, 0.5);
    }
    public void far_blue_right(){
        robot_move(-20, 0.5);
        sleep(150);
        robot_turn(60);
        sleep(150);
        robot_move(-11, 0.5);
        sleep(150);
        robot_move(11, 0.5);
        sleep(150);
        robot_turn(-60);
        sleep(150);
        robot_move(-30, 0.5);
        sleep(150);
        robot_turn(93);
        sleep(150);
        robot_move(-86, 0.5);
    }

    // helper functions ----- ( ๑‾̀◡‾́)σ"
    public void setLeftMotorPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
    }
    public void setRightMotorPower(double power) {
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
    public void setMotorPower(double rightFrontPower, double rightBackPower, double leftFrontPower, double leftBackPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public double scalePIDOutput(double output, double min, double max) {
        double yup = output / PIDOUTPUTSCALE;
        double changeSign = 1;

        if (yup < 0) changeSign = -1;

        if (Math.abs(yup) < min) {
            return min * changeSign;
        } else if (Math.abs(yup) > max) {
            return max * changeSign;
        } else {
            return output / PIDOUTPUTSCALE;
        }
    }

    // move functions ----- (っ＾▿＾)۶🍸🌟🍺٩(˘◡˘ )
    public void robot_turn(double degrees) {
        imu.initialize();
        sleep(10);
        imu.resetYaw();
        double temp = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetHeading = getHeading() + degrees;
        double error = targetHeading - getHeading();
        double derivative = error - turn_previousError;

        while (Math.abs(error) > 0.1 || derivative > 0.2) { // tolerance in degrees, adjust as needed
            //while (Math.abs(error) > 0.075 || derivative > 0.2) { // tolerance in degrees, adjust as needed

                //pid
            error = targetHeading - getHeading();                   // error
            derivative = error - turn_previousError;         // derivative
            if (Math.abs(error) < 17.5) turn_integral += error;     // integral

            telemetry.addData("derivative", derivative);
            telemetry.addData("error", error);
            telemetry.addData("turn_integral", turn_integral);
            telemetry.addData("target", targetHeading);




            //output and scale
            double output = (turnKp * error) + (turnKi * turn_integral) + (turnKd * derivative);
            telemetry.addData("output before scale", output);
            output = scalePIDOutput(output, 0.075, 1000);
            telemetry.addData("output after scale", output);

            //set power to motors
            setLeftMotorPower(-output);
            setRightMotorPower(output);

            //prep for next loop
            turn_previousError = error;
            telemetry.update();
            sleep(20);
        }
        // Stop motors after turning
        setLeftMotorPower(0);
        setRightMotorPower(0);

        //reset variables
        turn_previousError = 0;
        turn_integral = 0;

        imu.resetYaw();
    }

    public void robot_move(double distance, double maxPower) {

        double targetDistanceTicks = distance * ticks_per_inch /2;
        //double averageEncoder = (leftFront.getCurrentPosition() + rightFront.getCurrentPosition()) / (2.0);
        double averageEncoder = (leftFront.getCurrentPosition() + leftBack.getCurrentPosition()*(-1)) / (2.0);

        double errorDist;
        double gostraightL;
        double gostraightR;

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //this is right side encoder!!!
        //rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //this is left side encoder!!!
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        errorDist = targetDistanceTicks - averageEncoder;
        double derivativeDist = errorDist - previousErrorDist;

        while (Math.abs(targetDistanceTicks - averageEncoder) > 186.253 ) { // 186.235~0.1 inch, Tolerance in ticks, adjust as needed
            //averageEncoder = ((leftFront.getCurrentPosition() + rightFront.getCurrentPosition()) / 2.0);
            averageEncoder = ((leftFront.getCurrentPosition() + leftBack.getCurrentPosition()*(-1)) / 2.0);

            // Distance PID
            errorDist = targetDistanceTicks - averageEncoder;                       // error
            derivativeDist = errorDist - previousErrorDist;                  // derivative
            if (Math.abs(errorDist) < ticks_per_inch*2) integralDist += errorDist;  // integral

            // output and scale
            double outputDist = (Kp_dist * errorDist) + (Ki_dist * integralDist) + (Kd_dist * derivativeDist);
            outputDist = outputDist / encoder_tick_scale;
            if((leftFront.getCurrentPosition() - leftBack.getCurrentPosition()*(-1))>90) //right faster
            {
                gostraightL=0.01;
                gostraightR=-0.01;
            }
            else if((leftFront.getCurrentPosition() - leftBack.getCurrentPosition()*(-1))<-90) //right slower
            {
                gostraightL=-0.01;
                gostraightR=0.01;
            }
            else
            {
                gostraightL=0;
                gostraightR=0;
            }
            double leftPower = scalePIDOutput(outputDist , 0.05, maxPower)+gostraightL;
            double rightPower = scalePIDOutput(outputDist , 0.05, maxPower)+gostraightR;

            telemetry.addData("derivative", derivativeDist);
            telemetry.addData("targetDistanceTicks", targetDistanceTicks);
            telemetry.addData("errorDist", errorDist);
            telemetry.addData("outputDist", outputDist);
            telemetry.addData("left F encoder", leftFront.getCurrentPosition());
            telemetry.addData("right F encoder", rightFront.getCurrentPosition());
            telemetry.addData("left B encoder", leftBack.getCurrentPosition());
            telemetry.addData("right B encoder", rightBack.getCurrentPosition());

            // set power to motors
            setLeftMotorPower(leftPower);
            setRightMotorPower(rightPower);

            previousErrorDist = errorDist;
            telemetry.update();
            sleep(10);
        }
        // Stop motors
        setLeftMotorPower(0);
        setRightMotorPower(0);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //this is right side encoder!!!
        //rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //this is left side encoder!!!
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reset variables
        previousErrorDist = 0;
        integralDist = 0;
        previousErrorHeading = 0;
        integralHeading = 0;
    }

    public void robot_strafe(double distance, double maxPower) {
        double targetDistanceTicks = distance * ticks_per_inch;
        imu.resetYaw();

        // reset encoder
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double middleEncoderPosition = rightBack.getCurrentPosition();

        double startHeading = getHeading();
        double errorStrafe, errorHeading;

        while ((Math.abs(targetDistanceTicks - middleEncoderPosition) > ticks_per_inch/16) || (Math.abs(startHeading - getHeading()) > 0.15)) { // Tolerance in ticks & degrees, adjust as needed
            middleEncoderPosition = rightBack.getCurrentPosition();


            // Strafe PID
            errorStrafe = targetDistanceTicks - middleEncoderPosition;                          // error
            double derivativeStrafe = errorStrafe - previousErrorStrafe;                        // derivative
            if (Math.abs(errorStrafe) < ticks_per_inch * 2) integralStrafe += errorStrafe;      // integral

            //output and scale
            double outputStrafe = (Kp_strafe * errorStrafe) + (Kd_strafe * derivativeStrafe) + (Ki_strafe * integralStrafe);
            outputStrafe = outputStrafe / encoder_tick_scale;

            // Heading PID
            errorHeading = startHeading - getHeading();                     // error
            double derivativeHeading = errorHeading - previousErrorHeading; // derivative
            integralHeading += errorHeading;                                // integral

            //output
            double outputHeading = (Kp_heading * errorHeading) + (Kd_heading * derivativeHeading) + (Ki_heading * integralHeading);

            double headingDirectionFactor = (distance > 0) ? -1 : 1;
            if (errorHeading < 0) headingDirectionFactor *= -1;

            // Combining outputs
            double frontPower = scalePIDOutput(outputStrafe - (outputHeading * headingDirectionFactor * heading_correct_scale), 0.075, maxPower);
            double backPower = scalePIDOutput(outputStrafe + (outputHeading * headingDirectionFactor * heading_correct_scale), 0.075, maxPower);

            // Set motor powers for strafing
            setMotorPower(frontPower, -backPower, -frontPower, backPower);

            previousErrorStrafe = errorStrafe;
            previousErrorHeading = errorHeading;

            sleep(15);

        }
        // Stop motors
        setMotorPower(0, 0, 0, 0);

        // reset encoders
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reset variables
        previousErrorStrafe = 0;
        integralStrafe = 0;
        previousErrorHeading = 0;
        integralHeading = 0;
    }

    //apriltag stuff --- --- --- --- --- --- --- --- --- --- ---
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        /*if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }*/
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public List<Double> lookyTag(){
        targetFound = false;
        desiredTag  = null;
        double maxloop = 1000;
        double currentLoop = 0;

        while (currentLoop < maxloop) {

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
                telemetry.update();
            }
            if (targetFound == true) break;
            currentLoop += 1;
        }

        double  rangeError      = desiredTag.ftcPose.range;
        double  headingError    = desiredTag.ftcPose.bearing;
        double  yawError        = desiredTag.ftcPose.yaw;

        List<Double> aprilValues = new ArrayList<>();
        aprilValues.add(rangeError);
        aprilValues.add(headingError);
        aprilValues.add(yawError);
        return aprilValues;
    }
}