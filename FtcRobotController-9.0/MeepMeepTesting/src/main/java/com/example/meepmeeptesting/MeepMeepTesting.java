package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 13)
                /*
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, 65, Math.toRadians(-90)))
                                .splineTo(new Vector2d(-29.5,40 ), Math.toRadians(-45))
                                .back(8)
                                .turn(Math.toRadians(45))
                                .strafeRight(35)
                                .lineTo(new Vector2d(-60,11.5))
                                .forward(70)
                                .splineTo(new Vector2d(52,42 ), Math.toRadians(0))
                                .build()

                 */
                /*
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, 65, Math.toRadians(-90)))
                                .splineTo(new Vector2d(-42.5,40 ), Math.toRadians(-135))
                                .back(8)
                                .turn(Math.toRadians(135))
                                .strafeRight(35)
                                .lineTo(new Vector2d(-60,11.5))
                                .forward(70)
                                .splineTo(new Vector2d(52,26 ), Math.toRadians(0))
                                .build()
                */
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, 65, Math.toRadians(-90)))
                                .forward(30)
                                .back(2)
                                .strafeRight(15)
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(-60,11.5))
                                .forward(70)
                                .splineTo(new Vector2d(52,36 ), Math.toRadians(0))
                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .start();
    }
}