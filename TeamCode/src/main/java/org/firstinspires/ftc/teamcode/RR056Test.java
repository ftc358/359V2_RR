package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class RR056Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12,-60,Math.toRadians(270));
        drive.setPoseEstimate(startPose);


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(18)
                .turn(Math.toRadians(45))
                .back(10)
                .forward(10)
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(50,-36))

                .lineToConstantHeading(new Vector2d(12,-12))
                .lineToConstantHeading(new Vector2d(-16,-12))
                .lineToConstantHeading(new Vector2d(12,-12))
                .lineToConstantHeading(new Vector2d(50,-36))
//
//                .lineToConstantHeading(new Vector2d(12,-12))
//                .lineToConstantHeading(new Vector2d(-16,-12))
//                .lineToConstantHeading(new Vector2d(12,-12))
//                .lineToConstantHeading(new Vector2d(50,-36))
//
//                .lineToConstantHeading(new Vector2d(12,-12))
//                .lineToConstantHeading(new Vector2d(-16,-12))
//                .lineToConstantHeading(new Vector2d(12,-12))
//                .lineToConstantHeading(new Vector2d(50,-36))
                .build();

        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(trajSeq);
        }



    }


}
