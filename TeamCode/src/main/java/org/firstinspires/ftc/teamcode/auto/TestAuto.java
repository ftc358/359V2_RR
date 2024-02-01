package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@Autonomous
public class TestAuto extends LinearOpMode {

    // PID constants for turning
    public static double turnKp = 0.45; //tuned 1/31
    public static double turnKi = 0.01; //tuned 1/30
    public static double turnKd = 0.06; //tuned 1/31
    // PID constants for forward/back
    public static double Kp_dist = 0.1; //tuned 1/30
    public static double Ki_dist = 0.006; //tuned 1/30
    public static double Kd_dist = 0.2; //tuned 1/30
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
        }

        // run auton here!! ٩(˘◡˘ )
        if (isStarted()) {

            robot_turn(90);
            sleep(1000);
            robot_turn(-90);
            sleep(1000);
            robot_turn(45);
            sleep(1000);
            robot_turn(-45);

            //far_red_left();
        }
    }

    // auton routes
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
        robot_move(-16, 0.5);
        sleep(150);
        robot_turn(-45);
        sleep(150);
        robot_move(-16, 0.5);
        sleep(150);
        robot_move(16, 0.5);
        sleep(150);
        robot_turn(45);
        sleep(150);
        robot_move(-37 , 0.5);
        sleep(150);
        robot_turn(-88);
        sleep(150);
        robot_move(-86, 0.5);
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

    // functions start here ----- (っ＾▿＾)۶🍸🌟🍺٩(˘◡˘ )
    public void robot_turn(double degrees) {
        double targetHeading = getHeading() + degrees;
        double error = targetHeading - getHeading();

        while (Math.abs(error) > 0.1) { // tolerance in degrees, adjust as needed
            //pid
            error = targetHeading - getHeading();                   // error
            double derivative = error - turn_previousError;         // derivative
            if (Math.abs(error) < 17.5) turn_integral += error;     // integral

            //output and scale
            double output = (turnKp * error) + (turnKi * turn_integral) + (turnKd * derivative);
            output = scalePIDOutput(output, 0.075, 1000);

            //set power to motors
            setLeftMotorPower(-output);
            setRightMotorPower(output);

            //prep for next loop
            turn_previousError = error;
            sleep(20);
        }
        // Stop motors after turning
        setLeftMotorPower(0);
        setRightMotorPower(0);

        //reset variables
        turn_previousError = 0;
        turn_integral = 0;
    }

    public void robot_move(double distance, double maxPower) {

        double targetDistanceTicks = distance * ticks_per_inch /2;
        double averageEncoder = (leftFront.getCurrentPosition() + rightFront.getCurrentPosition()) / (2.0);
        double errorDist;

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(targetDistanceTicks - averageEncoder) > 186.253) { // Tolerance in ticks, adjust as needed
            averageEncoder = ((leftFront.getCurrentPosition() + rightFront.getCurrentPosition()) / 2.0);

            // Distance PID
            errorDist = targetDistanceTicks - averageEncoder;                       // error
            double derivativeDist = errorDist - previousErrorDist;                  // derivative
            if (Math.abs(errorDist) < ticks_per_inch*2) integralDist += errorDist;  // integral

            // output and scale
            double outputDist = (Kp_dist * errorDist) + (Ki_dist * integralDist) + (Kd_dist * derivativeDist);
            outputDist = outputDist / encoder_tick_scale;

            double leftPower = scalePIDOutput(outputDist, 0.05, maxPower);
            double rightPower = scalePIDOutput(outputDist, 0.05, maxPower);

            // set power to motors
            setLeftMotorPower(leftPower);
            setRightMotorPower(rightPower);

            previousErrorDist = errorDist;

            sleep(10);
        }
        // Stop motors
        setLeftMotorPower(0);
        setRightMotorPower(0);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
}