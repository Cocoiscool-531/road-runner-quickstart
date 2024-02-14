package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(167.05832), Math.toRadians(167.05832), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(12, 62, Math.toRadians(-90)))
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
                .addEntity(myBot)
                .start();
    }
}