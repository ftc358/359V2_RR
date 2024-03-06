package org.firstinspires.ftc.teamcode.auto;
//for RR
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;


//copy from Auto
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
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
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
@Autonomous(name="test_diffy", group="Auto")

public class test_diffy extends LinearOpMode {

    IMU.Parameters para = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
    // hardware variables
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public IMU imu = null;
    public Servo claw1, claw2, diffy1, diffy2, wrist1, wrist2, intakePivot1, intakePivot2, intakeWrist;
    public DcMotorEx intake, horExt, lift;

    public ColorRangeSensor In1Color, In2Color; //Color Sensors for Intake

    //
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.

    private RedDetectionRight redDetectionRight;

    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public boolean targetFound = false;    // Set to true when an AprilTag target is detected

    public int side = 1;

    public void runOpMode() throws InterruptedException {

        // init phase
        if (opModeInInit()) {
            Robot robot = new Robot(hardwareMap);
            imu = robot.imu;
            claw1 = robot.claw1;
            claw2 = robot.claw2;

            diffy1 = robot.diffy1;
            diffy2 = robot.diffy2;

            wrist1 = robot.wrist1;
            wrist2 = robot.wrist2;

            lift = robot.lift;
            horExt = robot.horExt;
            intake = robot.intake;

            intakeWrist = robot.intakeWrist;
            intakePivot1 = robot.intakePivot1;
            intakePivot2 = robot.intakePivot2;

            In1Color=robot.In1Color;
            In2Color=robot.In1Color;



            //initAprilTag();
            //setManualExposure(6, 250);
        }
        while (opModeInInit()) {
            side = RedDetectionRight.getReadout();
            telemetry.addData("Side", side);
            telemetry.addData("LeftVal", RedDetectionRight.leftValue);
            telemetry.addData("CenterVal", RedDetectionRight.centerValue);
            telemetry.addData("RightVal", RedDetectionRight.rightValue);
            telemetry.update();
        }

        waitForStart();
        // run auton here!! ٩(˘◡˘ )
        if (isStarted()) {
            while(opModeIsActive()){
                telemetry.addData("In1Color(CM)", In1Color.getDistance(DistanceUnit.CM));
                telemetry.addData("In2Color(CM)", In2Color.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            /*sleep(200);
            grab();
            sleep(500);
            diffyHome();
            sleep(500);
            diffyBoard();
            sleep(500);
            lift.setPower(0.4);
            sleep(500);
            lift.setPower(-0.3);
            sleep(500);
            release();
            sleep(500);*/
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
}