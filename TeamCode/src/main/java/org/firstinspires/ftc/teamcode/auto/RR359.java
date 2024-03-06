package org.firstinspires.ftc.teamcode.auto;

//package org.firstinspires.ftc.teamcode.auto.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RR359 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //use the center of the backside of the robot as the pose
        //define the start position of the robot
        //Red far:  (-36, -64, Math.toRadians(0))
        //Red close: (36, -64, Math.toRadians(0))
        //Blue far: (-36, 64, Math.toRadians(180))
        //Blue close: (36,64, Math.toRadians(180))
        //Red center pixel drop (54, -36)
        //Red left pixel drop (54, -31)
        //Red right pixel drop (54, -41)
        //Blue center pixel drop (54, 36)
        //Blue left pixel drop (54, 31)
        //Blue right pixel drop (54, 41)

        Pose2d blue_far_pose = new Pose2d(-36, -64, Math.toRadians(-90));
        drive.setPoseEstimate(blue_far_pose);

        //1st trajectory, move backword to put the pixel on the center line
        Trajectory traj1 = drive.trajectoryBuilder(blue_far_pose)
                .back(31)
                .build();

        //2nd trajectory, strafe to leave the pixel
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-48,-48))
                //.forward(11)
                .build();

        //3nd trajectory, spline move around the pixel (back turn -- reserved: ture!!!)

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(),true)
                .splineTo(new Vector2d(-36, -12), Math.toRadians(0))
                .lineTo(new Vector2d(20,-12))
                .splineTo(new Vector2d(54, -36), Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        //give 1 sec to drop the pixel on board
        sleep(1000);
        //drop pose change (drop_x, drop_y, drop_angle)
        //traj3.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false) to get the new position for traj4 as start pose

        //4th trajectory back to pick more pixel
        //start from traj3 end pose plus drop pose changes
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0,0,Math.toRadians(0))),false)
                .splineTo(new Vector2d(20, -12), Math.toRadians(180))   //because the direction was reversed in traj3, now is 180 degree
                //.back(60)
                .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj4);

        //give intake 1 second, add intake code here
        sleep(1000);

        //intake pose change (in_x, in_y, in_angle)
        //traj4.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)

        //5th trajectory move to board second time
        //start from traj4 end pose plus intake pose changes
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0,0,Math.toRadians(0))),true)//reversed again!!!
                .lineToLinearHeading(new Pose2d(20, -12, Math.toRadians(180)))
                .splineTo(new Vector2d(54, -36), Math.toRadians(0))//I do not now why this has to be 0 degree
                .build();

        //move to board
        drive.followTrajectory(traj5);
    }
}
