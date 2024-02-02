package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

enum Route{
    RED_CLOSE_1,
    RED_CLOSE_2,
    RED_CLOSE_3,

    RED_FAR_1,
    RED_FAR_2,
    RED_FAR_3,

    BLUE_CLOSE_1,
    BLUE_CLOSE_2,
    BLUE_CLOSE_3,

    BLUE_FAR_1,
    BLUE_FAR_2,
    BLUE_FAR_3,

}

public class MeepMeepTesting {


    public static final Route ROUTE = Route.RED_CLOSE_1;
    public static final double DELAY = 0.5;
    public static final double MAXVEL = 50;
    public static final double MAXACCEL = 60;
    public static final double MAXANGAVEL = Math.toRadians(180);
    public static final double MAXANGACCEL = Math.toRadians(180);

    public static final double TRACKWIDTH = 12.70;

    public static void main(String[] args) throws Exception{
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAXVEL, MAXACCEL, MAXANGAVEL, MAXANGACCEL, TRACKWIDTH)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(270)))
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

                                .build()
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