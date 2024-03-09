package org.firstinspires.ftc.teamcode.auto;

//package org.firstinspires.ftc.teamcode.auto.roadrunner.drive.opmode;

import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.claw2GrabPos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.claw2ReleasePos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.clawGrabPos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.clawReleasePos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.diffyDropPos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.diffyHoldPos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.intakePivotDriving;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.intakePivotIntake;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.intakeWristDriving;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.vision.BlueDetectionLeft;
import org.firstinspires.ftc.teamcode.vision.RedDetectionRight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name="RR_Blue_Far_slow",group = "Auto")
public class RR_Blue_far_slow extends LinearOpMode {
    //Parameters for RR adjust (on dashboard)
    public static double slow_velocity = 18; //normal speed is 39
    public static double Final_angle = 1; //the drop pose angle adjust - use it to make sure the robot faces the board straightly.
    public static double Third_turn_x_i = -36; //the 3rd traj initial start pose x
    public static double end_pose1_y_adj = -1; //the first drop y adjust (positive - move left)
    public static double end_pose2_y_adj = 0; //the second drop y adjust (positive - move left)
    public static double intake_pose_x = -49; //the traj4 end pose x for intake
    public static double intake_pose_y = 11; //the traj4 end pose x for intake //12
    public static double end_pose_Y_i = 35.5; //end pose Y initial value
    double end_pose_Y = 0; //end pose Y - this is the value used in the traj
    double Third_turn_x = 0; //the 3rd traj start pose x - this is the value used in the traj

    //hardware - drive and operation
    public IMU imu = null;
    public Servo claw1, claw2, diffy1, diffy2, wrist1, wrist2, intakePivot1, intakePivot2, intakeWrist;
    public DcMotorEx intake, horExt, lift;
    public ColorRangeSensor In1Color, In2Color;

    //hardware - vision
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.

    private BlueDetectionLeft blueDetectionLeft;

    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public boolean targetFound = false;    // Set to true when an AprilTag target is detected

    public int side = 1;  //original auto code for side detection
    public static int target = 3; //for test purpose only !!!, target is 1,2, or 3.

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // init phase
        if (opModeInInit()) {
            Robot robot = new Robot(hardwareMap);
            imu = robot.imu;

            claw1 = robot.claw1; //claw1 and claw2 are for dropper
            claw2 = robot.claw2;

            diffy1 = robot.diffy1; //diffy1 and diffy2 are for dropper
            diffy2 = robot.diffy2;

            wrist1 = robot.wrist1;
            wrist2 = robot.wrist2;

            lift = robot.lift;
            horExt = robot.horExt;
            intake = robot.intake;

            intakeWrist = robot.intakeWrist;
            intakePivot1 = robot.intakePivot1;
            intakePivot2 = robot.intakePivot2;

            In1Color = robot.In1Color;
            In2Color = robot.In2Color;

            initAprilTag();
            setManualExposure(6, 250);

            //hardware init
            diffyHome();
            grab();
            wrist1.setPosition(0.0);
            wrist2.setPosition(0.0);
            intakePivot1.setPosition(intakePivotDriving);
            intakePivot2.setPosition(intakePivotDriving);
            intakeWrist.setPosition(intakeWristDriving);
        }
        while (opModeInInit()) {
            side = BlueDetectionLeft.getReadout(); //change this to variable "target"
            horExt.setPower(-0.1);  //hold the intake extension tight

            telemetry.addData("Side", side);
            telemetry.addData("LeftVal", BlueDetectionLeft.leftValue);
            telemetry.addData("CenterVal", BlueDetectionLeft.centerValue);
            telemetry.addData("RightVal", BlueDetectionLeft.rightValue);
            telemetry.update();
        }
        //if not camera installed, comment out these two lines!!!
        visionPortal.setProcessorEnabled(blueDetectionLeft, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        Pose2d Blue_far_pose = new Pose2d(-35.5, 62.75, Math.toRadians(90));
        Pose2d End_2nd = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(Blue_far_pose);
        //trajectory from left to the board;
        //trajectory for left target (1)
        //1st trajectory, spline (back) and turn 85 degree (clockwise) to put the pixel on the right line
        Trajectory traj1L = drive.trajectoryBuilder(Blue_far_pose, true)
                .splineTo(new Vector2d(-30, 36), Math.toRadians(-5),
                        SampleMecanumDrive.getVelocityConstraint(slow_velocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) //-36,-12
                .build();
        //2nd trajectory, move forward and spline to -100 degree to leave the pixel
        Trajectory traj2L = drive.trajectoryBuilder(traj1L.end())
                .lineToSplineHeading(new Pose2d(-42, 36, Math.toRadians(100)),
                        SampleMecanumDrive.getVelocityConstraint(slow_velocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        //trajectory for middle target (2)
        //1st trajectory, move (back) to put the pixel on the center line
        Trajectory traj1M = drive.trajectoryBuilder(Blue_far_pose)
                .back(30.5)
                .build();
        //2nd trajectory, strafe to leave the pixel
        Trajectory traj2M = drive.trajectoryBuilder(traj1M.end())
                .strafeTo(new Vector2d(-48, 48))
                //.forward(11)
                .build();
        //trajectory for right target (3)
        //1st trajectory, back and spline 45 degree (clockwise) to put the pixel on left line
        Trajectory traj1R = drive.trajectoryBuilder(Blue_far_pose, true)
                .lineToSplineHeading(new Pose2d(-41, 36, Math.toRadians(45)))
                .build();
        //2nd trajectory, move forward and turn the head to -100 degree (face to team)
        Trajectory traj2R = drive.trajectoryBuilder(traj1R.end())
                .lineToSplineHeading(new Pose2d(-36, 48, Math.toRadians(100)))
                .build();


        waitForStart();
        // run auton here!! ٩(˘◡˘ )
        //if (isStopRequested()) return;

        if (isStarted()) {
            telemetry.addData("ColSen1",In1Color.getDistance(DistanceUnit.INCH));
            telemetry.addData("ColSen2",In2Color.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(2000);

            intakePivot1.setPosition(intakePivotIntake);
            intakePivot2.setPosition(intakePivotIntake);
            intakeWrist.setPosition(0.2);
            sleep(500);
            horExt.setPower(0.4);
            sleep(500);
            horExt.setPower(0);
            intake.setPower(1);
            sleep(300);
            long startTime = System.currentTimeMillis();
            double scale = 0.01;
            while (((In1Color.getDistance(DistanceUnit.INCH)>1.4)&&(In2Color.getDistance(DistanceUnit.INCH)>1.4)) ){
                intakeWrist.setPosition(0.2-scale);
                scale += 0.01;
                sleep(100);
            }
            intake.setPower(0);
            sleep(500);
            horExt.setPower(-0.4);
            intakePivot1.setPosition(intakePivotDriving);
            intakePivot2.setPosition(intakePivotDriving);
            intakeWrist.setPosition(intakeWristDriving);
            sleep(2000);
//            intakePivot1.setPosition(0.11);
//            intakePivot2.setPosition(0.11);
//            intakeWrist.setPosition(0.18);
//            sleep(500);
//            horExt.setPower(0.4);
//            sleep(500);
//            horExt.setPower(0.1);
//            intake.setPower(1);
//            intakePivot1.setPosition(0.11);
//            intakePivot2.setPosition(0.11);
//            sleep(300);
//            intakePivot1.setPosition(0.1);
//            intakePivot2.setPosition(0.1);
//            sleep(100);
//            intakePivot1.setPosition(0.08);
//            intakePivot2.setPosition(0.08);
//            intakeWrist.setPosition(0.15);
//            sleep(120);
//            intakePivot1.setPosition(0.06);
//            intakePivot2.setPosition(0.06);
//            sleep(120);
//            intakePivot1.setPosition(0.05);
//            intakePivot2.setPosition(0.05);
//            sleep(100);
//            intakePivot1.setPosition(0.04);
//            intakePivot2.setPosition(0.04);
//            sleep(1000);
//            horExt.setPower(-0.4);
//            intakePivot1.setPosition(intakePivotDriving);
//            intakePivot2.setPosition(intakePivotDriving);
//            intakeWrist.setPosition(intakeWristDriving);
//            sleep(2000);
        }
    }

    public void grab() {
        claw1.setPosition(clawGrabPos);
        claw2.setPosition(claw2GrabPos);
    }

    public void release() {
        claw1.setPosition(clawReleasePos);
        claw2.setPosition(claw2ReleasePos);
    }

    public void diffyHome() {
        diffy1.setPosition(diffyHoldPos);
        diffy2.setPosition(diffyHoldPos);
    }

    public void diffyBoard() {
        diffy1.setPosition(diffyDropPos);
        diffy2.setPosition(diffyDropPos);
    }

    //The following three functions are from Original Code for AprilTag
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(463.566f, 463.566f, 316.402f, 176.412f)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);
        blueDetectionLeft = new BlueDetectionLeft(telemetry);//INIT THIS FOR RED SIDE AND COMMENT OUT BLUE

        /* From Config File for 640 by 360:
        * size="640 360"
            focalLength="463.566f, 463.566f"
            principalPoint="316.402f, 176.412f"
            distortionCoefficients="0.111626 , -0.255626, 0, 0, 0.107992, 0, 0, 0"
            />
        *
        * */
        // Create the vision portal by using a builder.
        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 360))
                .addProcessor(aprilTag)
                //.addProcessor(blueDetectionLeft
                .addProcessor(blueDetectionLeft)
                .build();

        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(blueDetectionLeft, true);


    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Not Ready");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public double lookyTag(int tagID, int tagReading) {
        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == tagID)) {
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
        double output = 0;
        if (targetFound) {
            double rangeError = desiredTag.ftcPose.range;
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;
            switch (tagReading) {
                case 0:
                    output = rangeError;
                    break;
                case 1:
                    output = headingError;
                    break;
                case 2:
                    output = yawError;
                    break;
            }
        }
        return output;
    }
}

