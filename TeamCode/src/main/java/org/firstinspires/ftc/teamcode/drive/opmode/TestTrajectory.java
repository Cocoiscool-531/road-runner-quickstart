package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TestTrajectory extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(90,0))
                .lineTo(new Vector2d(72, -48))
                .lineTo(new Vector2d(48, 0))
                .lineTo(new Vector2d(24, -48))
                .lineTo(new Vector2d(0, 0))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(24)
                .strafeRight(24)
                .forward(24)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(12, -24), Math.toRadians(0))
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(0)
                .build();

        List<TrajectorySequence> trajs = Arrays.asList(traj1, traj2, traj3, traj4);

        waitForStart();
        int runTraj = 0;
        while(runTraj == 0 && !isStopRequested()){
            if(gamepad1.dpad_up){runTraj = 1; break;}
            if(gamepad1.dpad_right){runTraj = 2; break;}
            if(gamepad1.dpad_down){runTraj = 3; break;}
            if(gamepad1.dpad_left){runTraj = 4; break;}
        }
        telemetry.addData("Trajectory: ", runTraj);
        telemetry.addLine("Press A To Run");
        telemetry.update();

        while(!gamepad1.a && !isStopRequested());

        if(!isStopRequested()){drive.followTrajectorySequence(trajs.get(runTraj - 1));}
    }
}
