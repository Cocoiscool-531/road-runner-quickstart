package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.MecanumDrive;


public class BlueNearLeft extends BaseAuto{

    Action leftPath;
    Action centerPath;
    Action rightPath;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));

        leftPath = drive.actionBuilder(drive.pose)
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
                .build();

        centerPath = drive.actionBuilder(drive.pose)
                .lineToY(33)
                .waitSeconds(3)
                .lineToY(40)
                .splineTo(new Vector2d(40, 36), Math.toRadians(-180))
                .lineToX(49)
                .waitSeconds(5)
                .lineToX(44)
                .splineTo(new Vector2d(48, 58), 0)
                .lineToX(60)
                .build();

        rightPath = drive.actionBuilder(drive.pose)
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
                .build();

        String propPos = detectPropPosition("blue");

        if(propPos.equals("Left")){
            Actions.runBlocking(leftPath);
        }
        else if(propPos.equals("Center")){
            Actions.runBlocking(centerPath);
        } else if(propPos.equals("Right")){
            Actions.runBlocking(rightPath);
        }

    }
}
