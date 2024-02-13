package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity blueNearLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(167.05832), Math.toRadians(167.05832), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))
                                .waitSeconds(5)
                                .splineTo(new Vector2d(33, 30), Math.toRadians(180))
                                .waitSeconds(3)
                                .splineToLinearHeading(new Pose2d(48, 36), 0)
                                .waitSeconds(3)
                                .lineTo(new Vector2d(48, 58))
                                .waitSeconds(30 - 16.87)
                                .lineTo(new Vector2d(12, 60))
                                .turn(Math.toRadians(-90))
                                .build()
                );
        RoadRunnerBotEntity blueFarRight = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(167.05832), Math.toRadians(167.05832), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(-180)))
                                        .lineTo(new Vector2d(0, -12))
                                        .splineToConstantHeading(new Vector2d(12, -24), Math.toRadians(-90))
                                        .turn(Math.toRadians(90))
                                        .turn(Math.toRadians(90))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(0, -36), Math.toRadians(-90))
                                        .build()
                                        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(blueNearLeft)
                .addEntity(blueFarRight)
                .start();
    }
}