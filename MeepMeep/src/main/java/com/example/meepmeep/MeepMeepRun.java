package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRun {
    public static void main(String[] args) {

        double MAX_VEL = 54.66855022514893;
        double MAX_ACCEL = 54.66855022514893;
        double MAX_ANG_VEL = Math.toRadians(174.0154);
        double MAX_ANG_ACCEL = Math.toRadians(174.0154);

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                                .lineTo(new Vector2d(0,96))
                                .lineTo(new Vector2d(48, 72))
                                .lineTo(new Vector2d(0, 48))
                                .lineTo(new Vector2d(48, 24))
                                .lineTo(new Vector2d(0, 0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}