package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.messages.*;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    public static class Params {
        
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // drive model parameters
        public double inPerTick = 1;
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx driveFrontLeft, driveBackLeft, driveBackRight, driveFrontRight;

    public final VoltageSensor voltageSensor;

    public final LazyImu imu;

    public final Localizer localizer;
    public Pose2d pose;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final Encoder driveFrontLeft, driveBackLeft, driveBackRight, driveFrontRight;

        private int lastdriveFrontLeftPos, lastdriveBackLeftPos, lastdriveBackRightPos, lastdriveFrontRightPos;
        private Rotation2d lastHeading;

        public DriveLocalizer() {
            driveFrontLeft = new OverflowEncoder(new RawEncoder(MecanumDrive.this.driveFrontLeft));
            driveBackLeft = new OverflowEncoder(new RawEncoder(MecanumDrive.this.driveBackLeft));
            driveBackRight = new OverflowEncoder(new RawEncoder(MecanumDrive.this.driveBackRight));
            driveFrontRight = new OverflowEncoder(new RawEncoder(MecanumDrive.this.driveFrontRight));

            // TODO: reverse encoders if needed
            //   driveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            lastdriveFrontLeftPos = driveFrontLeft.getPositionAndVelocity().position;
            lastdriveBackLeftPos = driveBackLeft.getPositionAndVelocity().position;
            lastdriveBackRightPos = driveBackRight.getPositionAndVelocity().position;
            lastdriveFrontRightPos = driveFrontRight.getPositionAndVelocity().position;

            lastHeading = Rotation2d.exp(imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair driveFrontLeftPosVel = driveFrontLeft.getPositionAndVelocity();
            PositionVelocityPair driveBackLeftPosVel = driveBackLeft.getPositionAndVelocity();
            PositionVelocityPair driveBackRightPosVel = driveBackRight.getPositionAndVelocity();
            PositionVelocityPair driveFrontRightPosVel = driveFrontRight.getPositionAndVelocity();

            FlightRecorder.write("MECANUM_ENCODERS", new MecanumEncodersMessage(
                    driveFrontLeftPosVel, driveBackLeftPosVel, driveBackRightPosVel, driveFrontRightPosVel));

            Rotation2d heading = Rotation2d.exp(imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (driveFrontLeftPosVel.position - lastdriveFrontLeftPos),
                            driveFrontLeftPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (driveBackLeftPosVel.position - lastdriveBackLeftPos),
                            driveBackLeftPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (driveBackRightPosVel.position - lastdriveBackRightPos),
                            driveBackRightPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (driveFrontRightPosVel.position - lastdriveFrontRightPos),
                            driveFrontRightPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastdriveFrontLeftPos = driveFrontLeftPosVel.position;
            lastdriveBackLeftPos = driveBackLeftPosVel.position;
            lastdriveBackRightPos = driveBackRightPosVel.position;
            lastdriveFrontRightPos = driveFrontRightPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        driveFrontLeft = hardwareMap.get(DcMotorEx.class, "driveFrontLeft");
        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "driveBackLeft");
        driveBackRight = hardwareMap.get(DcMotorEx.class, "driveBackRight");
        driveFrontRight = hardwareMap.get(DcMotorEx.class, "driveFrontRight");

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   driveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        driveFrontLeft.setPower(wheelVels.driveFrontLeft.get(0) / maxPowerMag);
        driveBackLeft.setPower(wheelVels.driveBackLeft.get(0) / maxPowerMag);
        driveBackRight.setPower(wheelVels.driveBackRight.get(0) / maxPowerMag);
        driveFrontRight.setPower(wheelVels.driveFrontRight.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                driveFrontLeft.setPower(0);
                driveBackLeft.setPower(0);
                driveBackRight.setPower(0);
                driveFrontRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double driveFrontLeftPower = feedforward.compute(wheelVels.driveFrontLeft) / voltage;
            double driveBackLeftPower = feedforward.compute(wheelVels.driveBackLeft) / voltage;
            double driveBackRightPower = feedforward.compute(wheelVels.driveBackRight) / voltage;
            double driveFrontRightPower = feedforward.compute(wheelVels.driveFrontRight) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, driveFrontLeftPower, driveBackLeftPower, driveBackRightPower, driveFrontRightPower
            ));

            driveFrontLeft.setPower(driveFrontLeftPower);
            driveBackLeft.setPower(driveBackLeftPower);
            driveBackRight.setPower(driveBackRightPower);
            driveFrontRight.setPower(driveFrontRightPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                driveFrontLeft.setPower(0);
                driveBackLeft.setPower(0);
                driveBackRight.setPower(0);
                driveFrontRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double driveFrontLeftPower = feedforward.compute(wheelVels.driveFrontLeft) / voltage;
            double driveBackLeftPower = feedforward.compute(wheelVels.driveBackLeft) / voltage;
            double driveBackRightPower = feedforward.compute(wheelVels.driveBackRight) / voltage;
            double driveFrontRightPower = feedforward.compute(wheelVels.driveFrontRight) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, driveFrontLeftPower, driveBackLeftPower, driveBackRightPower, driveFrontRightPower
            ));

            driveFrontLeft.setPower(feedforward.compute(wheelVels.driveFrontLeft) / voltage);
            driveBackLeft.setPower(feedforward.compute(wheelVels.driveBackLeft) / voltage);
            driveBackRight.setPower(feedforward.compute(wheelVels.driveBackRight) / voltage);
            driveFrontRight.setPower(feedforward.compute(wheelVels.driveFrontRight) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint,
                0.25, 0.1
        );
    }
}
