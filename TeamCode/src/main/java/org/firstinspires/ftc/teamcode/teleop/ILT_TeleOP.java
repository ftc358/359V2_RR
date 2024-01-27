package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.hardware.RobotConstants.*;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.threadopmode.*;

@TeleOp(name="LibrA+ TeleOP 2.0a", group="TeleOP")

public class ILT_TeleOP extends ThreadOpMode{
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    //Vars for Drive Thread
    private double x, y, turn, theta, power,sin,cos,lf,lb,rf,rb,max;
    private double turnScale = 1.0;
    private double powerScale = 1.0;
    //Vars for Motor Extension thread
    private double horExtPwr;
    private double liftPwr;
    private double intakePwr;
    //Vars for Sensor Thread
    private double backDistanceIN;
    private boolean horExtRetract;
    //Vars for Pixel Thread
    private double claw1Pos = 0;
    private double claw2Pos = 0;
    private double wristPos = 0;
    private double diffy1Pos = 0;
    private double diffy2Pos = 0;
    private double intakeWrist = 0;
    private double intakePivot = 0;
    //Vars for Led Thread
    private RevBlinkinLedDriver.BlinkinPattern currentPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    private RevBlinkinLedDriver.BlinkinPattern previousPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;

    private int grabState = 0;
    private int wristState = 0;
    private int diffyState = 0;

    private int intakeTransfer = 0;
    long halt0, halt1, halt2, halt3, halt4, halt5;
    private boolean intakeIn = false;
    private boolean intakeOut = false;
    private boolean liftBypass = false;

    private boolean retractVfy;
    private boolean detect;

    double In1Distance, In2Distance;
    int ledState = 0;
    String color;

    boolean blocked = false;





    @Override
    public void mainInit() {
        Robot Robot = new Robot(hardwareMap);

        TaskThread driveThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //Mecanum Drive Code From https://www.youtube.com/watch?v=gnSW2QpkGXQ
                x = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x*turnScale;
                theta = Math.atan2(y,x);
                power = Math.hypot(x,y)*powerScale;


                sin = Math.sin(theta-Math.PI/4);
                cos = Math.cos(theta-Math.PI/4);
                max = Math.max(Math.abs(sin),Math.abs(cos));

                lf = power*cos/max +turn;
                rf = power*sin/max -turn;
                lb = power*sin/max +turn;
                rb = power*cos/max -turn;

                if ((power + Math.abs(turn)) > 1) {
                    lf  /= power + Math.abs(turn);
                    rf  /= power + Math.abs(turn);
                    lb  /= power + Math.abs(turn);
                    rb  /= power + Math.abs(turn);
                }

                Robot.leftFront.setPower(lf);
                Robot.leftBack.setPower(lb);
                Robot.rightFront.setPower(rf);
                Robot.rightBack.setPower(rb);
            }
        });

        TaskThread sensorThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                backDistanceIN = Robot.backDistance.getDistance(DistanceUnit.INCH);
                horExtRetract = Robot.horReset.isPressed();

                In1Distance = Robot.In1Color.getDistance(DistanceUnit.INCH);
                In2Distance = Robot.In2Color.getDistance(DistanceUnit.INCH);


                if (detect){//Intake Pixel Detection
                if (In1Distance <1.4 && In2Distance<1.4){
                    ledState = 2;
                }
                else if (In1Distance < 1.4 || In2Distance <1.4){
                    ledState = 1;
                }else {
                    ledState = 0;
                }
                }

            }
        });

        TaskThread horExtThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //Horizontal Extension Triggering
                if (!retractVfy && diffyState == 2){
                    horExtPwr = -1;
                }
                else if ((currentGamepad2.left_trigger>0.2)){
                    retractVfy = false;
                    horExtPwr = currentGamepad2.left_trigger;
                    if (diffyState ==0){
                        diffyState =1;
                    }

                }
                else if (currentGamepad2.left_bumper && !horExtRetract){
                    retractVfy = false;
                    horExtPwr = -1;
                }else if (currentGamepad2.left_bumper)
                {
                    retractVfy = false;
                    horExtPwr = -0.2;
                }else if (horExtRetract|| retractVfy){
                    retractVfy = true;
                    horExtPwr = -0.2;
                }else if (!retractVfy){
                    horExtPwr = 0;
                }else {
                    horExtPwr = 0;
                }

                //lift Trigerring
                if (liftBypass){
                    liftPwr = 0.7;
                }
                else if (currentGamepad2.right_trigger>0.1){
                    liftPwr = currentGamepad2.right_trigger;
                }else if (currentGamepad2.right_bumper){
                    liftPwr = -0.6;
                }else{
                    liftPwr = 0.1;
                }




                if ((currentGamepad2.left_trigger>0 || intakeIn)){

                    intakePwr = 1;
                }else if ((currentGamepad2.dpad_left || intakeOut)){
                    intakePwr = -1;
                }else{
                    intakePwr = 0;
                }

                //Set Power to Motor
                Robot.horExt.setPower(horExtPwr);
                Robot.lift.setPower(liftPwr);
                Robot.intake.setPower(intakePwr);
            }
        });


        TaskThread pixelMovement = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                switch (diffyState){
                    case 0: //Home position for Driving
                        blocked = false;
                        if (gamepad2.x){ledState = 6;
                         wristState = 3;
                        }else {
                            ledState = 5;
                            wristState = 0;
                        }
                        intakeTransfer = 0;
                        break;
                    case 1:
                        intakeTransfer = 1;
                        halt3 = 0;
                        detect = true;
                        break;
                    case 2:
                        ledState = 3;
                        blocked = true;//Horzital Extension Reverses
                        halt1 = 0;
                        intakeIn = true;
                        intakeTransfer = 2;
                        if (horExtRetract && (halt3 == 0)){
                            halt3 = System.currentTimeMillis();
                        }
                        else if (horExtRetract && (System.currentTimeMillis()-halt3)>400){
                            diffyState = 3;
                        }
                        halt0 = 0;
                        break;
                    case 3:
                        if (halt0 == 0){ //Intake goes up
                            halt0 = System.currentTimeMillis();
                        }
                        if ((System.currentTimeMillis()-halt0)>400) {//Wrist goes up
                            wristState = 1;
                            diffyState = 4;
                        }
                        break;
                    case 4: // GO back
                        if (halt1 == 0){ //Intake Transfer
                            halt1 = System.currentTimeMillis();
                        }
                        if ((System.currentTimeMillis()-halt1)>500) {
                            intakeIn = false;
                            intakeOut = true;
                            diffyState = 5;
                        }
                        halt2 = 0;
                        break;
                    case 5:
                        if (halt2 == 0){ //Retract and Arm
                            halt2 = System.currentTimeMillis();
                        }
                        if ((System.currentTimeMillis()-halt2)>400) {
                            intakeOut = false;
                            wristState = 2;
                            blocked = false;
                        }
                        break;
                    case 6: //Ready to drop
                        ledState = 4;
                        wristState = 3;
                        break;
                    case 7: //Drops here
                        wristState = 4;//drops
                        intakeTransfer = 0;

                        break;
                }





                switch (grabState){
                    case 0: //Not Grabbing
                        claw1Pos = clawReleasePos;
                        claw2Pos = claw2ReleasePos;
                        break;
                    case 1: //Grabbing
                        claw1Pos = clawGrabPos;
                        claw2Pos = claw2GrabPos;
                        break;
                }

                switch (wristState){
                    case 0: // Hold Position
                        diffy1Pos = diffyHoldPos;
                        diffy2Pos = diffyHoldPos;
                        wristPos = wristHoldPos;
                        grabState = 0;
                        break;

                    case 1: // Intake Position
                        grabState = 0;
                        wristPos = wristIntakePos;
                        diffy1Pos = diffyIntakePos;
                        diffy2Pos = diffyIntakePos;
                        break;

                    case 2: // Armed Position
                        wristPos = wristHoldPos;
                        diffy1Pos = diffyHoldPos;
                        diffy2Pos = diffyHoldPos;
                        break;

                    case 3: //Dropping Position
                        grabState = 1;

                        wristPos = wristDropPos+gamepad1.right_trigger*wristInvK_Scale;
                        diffy1Pos = diffyDropPos+gamepad1.right_trigger*diffyInvK_Scale;
                        diffy2Pos = diffyDropPos+gamepad1.right_trigger*diffyInvK_Scale;
                        break;
                    case 4: //Drops
                        wristPos = wristDropPos+gamepad1.right_trigger*wristInvK_Scale;
                        diffy1Pos = diffyDropPos+gamepad1.right_trigger*diffyInvK_Scale;
                        diffy2Pos = diffyDropPos+gamepad1.right_trigger*diffyInvK_Scale;
                        grabState = 0;
                }


                switch (intakeTransfer){
                    case 0://Ground Float to prevent driving impediment
                        intakePivot = intakePivotDriving;
                        intakeWrist = intakeWristDriving;
                        break;
                    case 1://Ground Tangent Intake
                        intakePivot = intakePivotIntake;
                        intakeWrist = intakeWristIntake;
                        break;
                    case 2://Transfers
                        intakePivot = intakePivotTransfer;
                        intakeWrist = intakeWristTransfer;
                        break;
                }

                Robot.claw1.setPosition(claw1Pos);
                Robot.claw2.setPosition(claw2Pos);
                Robot.wrist1.setPosition(wristPos);
                Robot.wrist2.setPosition(wristPos);
                Robot.intakePivot1.setPosition(intakePivot);
                Robot.intakePivot2.setPosition(intakePivot);
                Robot.intakeWrist.setPosition(intakeWrist);
                Robot.diffy1.setPosition(diffy1Pos);
                Robot.diffy2.setPosition(diffy2Pos);
            }
        });

        TaskThread ledThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {

                switch (ledState){
                    case 0://No Pixels esta en el intake;
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                        break;
                    case 1://Un pixel esta en el intake
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                        break;
                    case 2://Hay dos pixeles en el intake
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                        break;
                    case 3://Transfer In progress
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;
                        break;
                    case 4://Ready to droppy
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
                        break;
                    case 5: //Idle
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                        break;
                    case 6://Climb
                        currentPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST;
                }

                if (!currentPattern.equals(previousPattern)){
                    Robot.led.setPattern(currentPattern);
                    previousPattern = currentPattern;
                }//Reduces Calls

            }});

        TaskThread templateThread = new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
            }});

        registerThread(driveThread);
        registerThread(sensorThread);
        registerThread(horExtThread);
        registerThread(pixelMovement);
        registerThread(ledThread);
    }

    @Override
    public void mainLoop() {
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if ((currentGamepad2.a && !previousGamepad2.a) && !blocked){
            diffyState = (diffyState + 1)%8;}

        if ((currentGamepad1.right_bumper && !previousGamepad1.right_bumper) && diffyState == 6) {
            diffyState = 7;
        }

        if (gamepad2.x){
            diffyState = 0;
            ledState = 6;
            Robot.climb.setPower(-gamepad2.right_stick_y);
        }



        telemetry.addData("diffyState",diffyState);
        telemetry.addData("grabState",grabState);
        telemetry.addData("wristState",wristState);


        telemetry.addData("intakepivot",gamepad1.left_trigger);//0.67
        telemetry.addData("intakeWrist",gamepad1.right_trigger);//0.70
        telemetry.addData("diffyPos",diffy1Pos);
        telemetry.addData("BackDistance Reading (in) ",backDistanceIN);

        telemetry.addLine("Sensors");
        telemetry.addData("In1Distance",In1Distance); // Anything within 1.3in counts as in
        telemetry.addData("Color",color);

        telemetry.addData("LiftPower",liftPwr);





        telemetry.update();

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

    }


}
