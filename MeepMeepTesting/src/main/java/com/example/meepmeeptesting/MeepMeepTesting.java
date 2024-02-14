package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(167.05832), Math.toRadians(167.05832), 15)
                .build();

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(167.05832), Math.toRadians(167.05832), 15)
                .build();

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(167.05832), Math.toRadians(167.05832), 15)
                .build();

        myBot1.runAction(
                myBot1.getDrive().actionBuilder(new Pose2d(12, 62, Math.toRadians(-90)))
                        .strafeTo(new Vector2d(36, 62))
                        .splineTo(new Vector2d(36, 28), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(30, 28), Math.toRadians(-90))
                        .waitSeconds(3)
                        .lineToX(36)
                        .splineTo(new Vector2d(40, 40), Math.toRadians(180))
                        .lineToX(49)
                        .waitSeconds(5)
                        .lineToX(45)
                        .splineTo(new Vector2d(48, 58), 0)
                        .lineToX(60)
                        .build());
        myBot2.runAction(
                myBot2.getDrive().actionBuilder(new Pose2d(12, 62, Math.toRadians(-90)))
                        .lineToY(33)
                        .waitSeconds(3)
                        .lineToY(40)
                        .splineTo(new Vector2d(40, 36), Math.toRadians(-180))
                        .lineToX(49)
                        .waitSeconds(5)
                        .lineToX(44)
                        .splineTo(new Vector2d(48, 58), 0)
                        .lineToX(60)
                        .build());

        myBot3.runAction(
                myBot3.getDrive().actionBuilder(new Pose2d(12, 62, Math.toRadians(-90)))
                        .lineToY(34)
                        .turnTo(Math.toRadians(-180))
                        .lineToX(9)
                        .waitSeconds(3)
                        .lineToX(15)
                        .splineTo(new Vector2d(40, 28), Math.toRadians(-180))
                        .lineToX(49)
                        .waitSeconds(5)
                        .lineToX(44)
                        .splineTo(new Vector2d(48, 58), 0)
                        .lineToX(60)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot1)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .start();
    }
}