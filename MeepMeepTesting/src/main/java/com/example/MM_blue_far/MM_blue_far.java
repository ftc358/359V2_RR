package com.example.MM_blue_far;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

enum Route{
    RED_CLOSE,

    RED_FAR,

    BLUE_CLOSE,

    BLUE_FAR,

}

public class MM_blue_far {
    public static final Route ROUTE = Route.BLUE_FAR;
    public static final double DELAY = 0.5;
    public static final double MAXVEL = 38;
    public static final double MAXACCEL = 25;
    public static final double MAXANGAVEL = Math.toRadians(214.5843);
    public static final double MAXANGACCEL = Math.toRadians(60);
    public static final double TRACKWIDTH = 12.179;


    //Blue side parameters
    public static double Final_angle = 1; //the drop pose angle adjust - use it to make sure the robot faces the board straightly.
    public static double Third_turn_x_i = -36; //the 3rd traj initial start pose x
    public static double end_pose1_y_adj = -4; //the first drop y adjust (positive - move left)
    public static double end_pose2_y_adj = 0; //the second drop y adjust (positive - move left)
    public static double intake_pose_x = -53; //the traj4 end pose x for intake
    public static double intake_pose_y = 11; //the traj4 end pose x for intake //12
    public static double end_pose_Y_i = 35.5; //end pose Y initial value
    public static double end_pose_Y = 0; //end pose Y - this is the value used in the traj
    public static double Third_turn_x = 0; //the 3rd traj start pose x - this is the value used in the traj
    public static void main(String[] args) throws Exception{
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d Blue_far_pose = new Pose2d(-35.5, 62.75, Math.toRadians(90));
        Pose2d Blue_close_pose = new Pose2d(-35.5, 62.75, Math.toRadians(90));
        Pose2d Red_far_pose = new Pose2d(-35.5, 62.75, Math.toRadians(90));
        Pose2d Red_close_pose = new Pose2d(-35.5, 62.75, Math.toRadians(90));
        int target =1;
        switch (target) {
            case 1: //left
                end_pose_Y = end_pose_Y_i + 6;
                Third_turn_x = Third_turn_x_i + 10;
                break;
            case 2: //middle
                //trajectory from middle to the board;
                end_pose_Y = end_pose_Y_i;
                Third_turn_x = Third_turn_x_i;
                break;
            case 3: //right
                end_pose_Y = end_pose_Y_i - 6;
                Third_turn_x = Third_turn_x_i + 10;
                break;
        }

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGAVEL, MAXANGACCEL, TRACKWIDTH)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Blue_far_pose)
                                .setReversed(true) //first trajectory
                                .splineTo(new Vector2d(-30, 36), Math.toRadians(-5))
                                .setReversed(false) //second trajectory
                                .lineToSplineHeading(new Pose2d(-42, 36, Math.toRadians(100)))
                                .setReversed(true) //third trajectory
                                .splineTo(new Vector2d(Third_turn_x, 12), Math.toRadians(0)) //-36,-12
                                .lineTo(new Vector2d(20, 12))
                                .splineTo(new Vector2d(50, end_pose_Y + end_pose1_y_adj), Math.toRadians(Final_angle)) //50,-34.5
                                .setReversed(false) //fourth trajectory
                                .splineTo(new Vector2d(20, 11), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(intake_pose_x, intake_pose_y, Math.toRadians(180)))
                                .setReversed(true) //fifth trajectory
                                .lineToLinearHeading(new Pose2d(20, 11, Math.toRadians(180)))
                                .splineTo(new Vector2d(49, end_pose_Y + end_pose2_y_adj), Math.toRadians(Final_angle))
                                .build()
                                /*drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(270)))
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

                                .lineToConstantHeading(new Vector2d(12,-12))
                                .lineToConstantHeading(new Vector2d(-16,-12))
                                .lineToConstantHeading(new Vector2d(12,-12))
                                .lineToConstantHeading(new Vector2d(50,-36))

                                .lineToConstantHeading(new Vector2d(12,-12))
                                .lineToConstantHeading(new Vector2d(-16,-12))
                                .lineToConstantHeading(new Vector2d(12,-12))
                                .lineToConstantHeading(new Vector2d(50,-36))

                                .build()*/
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}