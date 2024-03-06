package org.firstinspires.ftc.teamcode.auto;

//package org.firstinspires.ftc.teamcode.auto.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;
//from auto
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.claw2GrabPos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.claw2ReleasePos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.clawGrabPos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.clawReleasePos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.diffyDropPos;
import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.diffyHoldPos;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import org.firstinspires.ftc.teamcode.vision.RedDetectionRight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name="RR_Red_Far_2",group = "Auto")
public class RR_Red_far_2RUN extends LinearOpMode {
    //Parameters for RR adjust (on dashboard)
    public static double Final_angle = 3; //the drop pose angle adjust - use it to make sure the robot faces the board straightly.
    public static double Third_turn_x_i = -36; //the 3rd traj initial start pose x
    public static double end_pose1_y_adj = 1; //the first drop y adjust (positive - move left)
    public static double end_pose2_y_adj = 2; //the second drop y adjust (positive - move left)
    public static double intake_pose_x = -49; //the traj4 end pose x for intake
    public static double intake_pose_y = -12; //the traj4 end pose x for intake
    public static double end_pose_Y_i = -35.5; //end pose Y initial value
    double end_pose_Y = 0; //end pose Y - this is the value used in the traj
    double Third_turn_x = 0; //the 3rd traj start pose x - this is the value used in the traj

    //hardware - drive and operation
    public IMU imu = null;
    public Servo claw1, claw2, diffy1, diffy2, wrist1, wrist2, intakePivot1, intakePivot2, intakeWrist;
    public DcMotorEx intake, horExt, lift;

    //hardware - vision
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.

    private RedDetectionRight redDetectionRight;

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

            //initAprilTag();
            //setManualExposure(6, 250);
        }
        while (opModeInInit()) {
            side = RedDetectionRight.getReadout(); //change this to variable "target"
            telemetry.addData("Side", side);
            telemetry.addData("LeftVal", RedDetectionRight.leftValue);
            telemetry.addData("CenterVal", RedDetectionRight.centerValue);
            telemetry.addData("RightVal", RedDetectionRight.rightValue);
            telemetry.update();
        }
        //if not camera installed, comment out these two lines!!!
        //visionPortal.setProcessorEnabled(redDetectionRight, false);
        //visionPortal.setProcessorEnabled(aprilTag, true);

        Pose2d Red_far_pose = new Pose2d(-35.5, -62.75, Math.toRadians(-90));
        Pose2d End_2nd = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(Red_far_pose);
        //trajectory from left to the board;
        //trajectory for left target (1)
        //1st trajectory, back and spline 45 degree (clockwise) to put the pixel on left line
        Trajectory traj1L = drive.trajectoryBuilder(Red_far_pose, true)
                .lineToSplineHeading(new Pose2d(-41, -36, Math.toRadians(-45)))
                .build();
        //2nd trajectory, move forward and turn the head to -100 degree (face to team)
        Trajectory traj2L = drive.trajectoryBuilder(traj1L.end())
                .lineToSplineHeading(new Pose2d(-36, -48, Math.toRadians(-100)))
                .build();

        //trajectory for middle target (2)
        //1st trajectory, move (back) to put the pixel on the center line
        Trajectory traj1M = drive.trajectoryBuilder(Red_far_pose)
                .back(30.5)
                .build();
        //2nd trajectory, strafe to leave the pixel
        Trajectory traj2M = drive.trajectoryBuilder(traj1M.end())
                .strafeTo(new Vector2d(-48, -48))
                //.forward(11)
                .build();
        //trajectory for right target (3)
        //1st trajectory, spline (back) and turn 85 degree (clockwise) to put the pixel on the right line
        Trajectory traj1R = drive.trajectoryBuilder(Red_far_pose, true)
                .splineTo(new Vector2d(-30, -36), Math.toRadians(5)) //-36,-12
                //.lineToSplineHeading(new Pose2d(-30,-36,Math.toRadians(-135)))
                .build();
        //2nd trajectory, move forward and spline to -100 degree to leave the pixel
        Trajectory traj2R = drive.trajectoryBuilder(traj1R.end())
                .lineToSplineHeading(new Pose2d(-42, -36, Math.toRadians(-100)))
                //.forward(14)
                .build();

        waitForStart();
        // run auton here!! ٩(˘◡˘ )
        //if (isStopRequested()) return;

        if (isStarted()) {
            switch (target) {
                case 1: //left
                    drive.followTrajectory(traj1L);
                    drive.followTrajectory(traj2L);
                    End_2nd = traj2L.end();
                    end_pose_Y = end_pose_Y_i + 6;
                    Third_turn_x = Third_turn_x_i + 10;
                    break;
                case 2: //middle
                    //trajectory from middle to the board;
                    drive.followTrajectory(traj1M);
                    drive.followTrajectory(traj2M);
                    End_2nd = traj2M.end();
                    end_pose_Y = end_pose_Y_i;
                    Third_turn_x = Third_turn_x_i;
                    break;
                case 3: //right
                    drive.followTrajectory(traj1R);
                    drive.followTrajectory(traj2R);
                    End_2nd = traj2R.end();
                    end_pose_Y = end_pose_Y_i - 6;
                    Third_turn_x = Third_turn_x_i + 10;
                    break;
            }

            //3nd trajectory, spline move around the pixel (back turn -- reserved: true!!!)
            Trajectory traj3 = drive.trajectoryBuilder(End_2nd, true)
                    .splineTo(new Vector2d(Third_turn_x, -12), Math.toRadians(0)) //-36,-12
                    .lineTo(new Vector2d(20, -12))
                    .splineTo(new Vector2d(50, end_pose_Y + end_pose1_y_adj), Math.toRadians(Final_angle)) //50,-34.5
                    .build();
            drive.followTrajectory(traj3);
            //all three targets share the following actions and trajectory!!!
            //give 1 sec to drop the first pixel on board
            sleep(150);
            diffyBoard();
            sleep(600);
            release();
            sleep(250);
            diffyHome();

            //drop pose may be changed to (drop_x, drop_y, drop_angle)
            //traj3.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false) to get the new position for traj4 as start pose
            //4th trajectory back to pick up 2nd round pixel
            //start from traj3 end pose plus drop pose changes
            Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)
                    .splineTo(new Vector2d(20, -12), Math.toRadians(180))   //because the direction was reversed in traj3, now is 180 degree
                    //.back(60)
                    .lineToLinearHeading(new Pose2d(intake_pose_x, intake_pose_y, Math.toRadians(180)))
                    .build();
            drive.followTrajectory(traj4);

            //give intake 2 second, add intake code here!!!!!
            //if there is enough time, put transfer code here too.
            //if there is no time for transfer, add transfer in the traj5
            sleep(1000);
            sleep(1000);

            //intake pose may be changed to (in_x, in_y, in_angle)
            //traj4.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)

            //5th trajectory move to board 2nd time
            //start from traj4 end pose plus intake pose changes
            Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(0))), true)//reversed again!!!
                    .lineToLinearHeading(new Pose2d(20, -12, Math.toRadians(180)))
                    .splineTo(new Vector2d(49, end_pose_Y + end_pose2_y_adj), Math.toRadians(Final_angle))//I do not now why this has to be 0 degree, 49,-33.5
                    .build();
            //move to board
            drive.followTrajectory(traj5);
            //drop the pixels
            sleep(150);
            diffyBoard();
            sleep(600);
            release();
            sleep(250);
            diffyHome();
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
//        blueDetectionLeft = new BlueDetectionLeft(telemetry);INIT THIS FOR RED SIDE AND COMMENT OUT BLUE
        redDetectionRight = new RedDetectionRight(telemetry);

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
                .addProcessor(redDetectionRight)
                .build();

        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(redDetectionRight, true);


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

