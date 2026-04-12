package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.commands.GetTurretToHub;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX motor;

    private final double gearBoxRatio = 9.0;
    private final StatusSignal<Angle> motorPosition;

    // Physical turret limits in degrees (robot-relative, with the -180° correction
    // for the turret facing the back of the robot). Derived from real encoder data.
    private static final double TURRET_MIN_ANGLE = -143.3;
    private static final double TURRET_MAX_ANGLE = 63.95;

    // Hub target coordinates (field frame, meters)
    public double TargetXposition = 4.625594;
    public double TargetYposition = 4.03479;

    // Turret mount offset from robot center (robot frame, meters)
    public double XofTurretOnBot = -0.05;
    public double YofTurretOnBot = -0.05;

    // Camera position relative to robot center (robot frame, meters)
    public double XofCameraOnBot = 0.0;
    public double YofCameraOnBot = 0.0;

    // Lob shot target coordinates (field frame, meters).
    // Used when the hub is not lit and the robot is collecting in mid-field.
    // The target is placed inside the alliance end zone, just past the hub, so
    // the ball arcs around the hub structure and stays within the playing field.
    // Tune these per alliance if the landing zone needs adjustment.
    private static final double LOB_TARGET_BLUE_X = 1.5;
    private static final double LOB_TARGET_BLUE_Y = 4.03479;
    private static final double LOB_TARGET_RED_X = 15.0;
    private static final double LOB_TARGET_RED_Y = 4.03479;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Notifier dashboardNotifier;

    public TurretSubsystem(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
        motor = new TalonFX(18);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = 7;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0.1;
        cfg.Slot0.kV = .12;
        cfg.Slot0.kA = .01;
        cfg.Slot0.kS = .25;

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 7;
        mm.MotionMagicAcceleration = 80;
        mm.MotionMagicJerk = 1600;
        motor.getConfigurator().apply(cfg);

        motorPosition = motor.getPosition();

        dashboardNotifier = new Notifier(this::updateDashboard);
        dashboardNotifier.startPeriodic(0.5);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(motorPosition);
        SmartDashboard.putNumber("Turret Angle (deg)", getTurretRotation());
    }

    // -------------------------------------------------------------------------
    // Pose helpers
    // -------------------------------------------------------------------------

    public double getPoseEstimatorY() {
        return poseEstimator.getEstimatedPosition().getY();
    }

    public double getPoseEstimatorX() {
        return poseEstimator.getEstimatedPosition().getX();
    }

    public double getPoseEstimatorRotation() {
        return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    // -------------------------------------------------------------------------
    // Angle / encoder conversion
    // -------------------------------------------------------------------------

    private double degreesToRadians(double degrees) {
        return degrees * (Math.PI / 180);
    }

    /**
     * Converts a desired turret angle (degrees, robot-relative) to the motor
     * encoder position (rotations). Empirically measured — accounts for the actual
     * zero offset and the motor's direction relative to the turret.
     *
     * Zero offset (0.3256835 rot) is the motor position when the turret is centered.
     * The negative slope means positive turret angles move the motor in reverse.
     */
    public double turretDegreesAndEncoderUnits(double degrees) {
        return (-0.0256402 * degrees) + 0.3256835;
    }

    private double encoderUnitsToDegrees(double encoderUnits) {
        return 360.0 * (encoderUnits / gearBoxRatio);
    }

    /** Returns the current turret angle in degrees using the cached motor position. */
    public double getTurretRotation() {
        return encoderUnitsToDegrees(motorPosition.getValueAsDouble());
    }

    /** Normalizes an angle to the range [-180, 180] degrees. */
    private double normalizeAngle(double degrees) {
        double angle = degrees;
        while (angle > 180)  angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /** Clamps a turret angle to the physical range [TURRET_MIN_ANGLE, TURRET_MAX_ANGLE]. */
    private double clampTurretAngle(double degrees) {
        return Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, degrees));
    }

    // -------------------------------------------------------------------------
    // Core aiming calculation — single source of truth
    // -------------------------------------------------------------------------

    /**
     * Calculates the motor encoder position needed to aim the turret at a given
     * field target. Handles:
     *   - Turret's field position (robot pose + mount offset)
     *   - Robot heading subtraction to convert field angle → robot-relative
     *   - −180° correction because the turret faces the back of the robot
     *   - Physical limit clamping
     *   - Empirical degree-to-encoder mapping
     *
     * All aiming commands funnel through here.
     *
     * @param targetX target X coordinate in the field frame (meters)
     * @param targetY target Y coordinate in the field frame (meters)
     * @return motor encoder position to pass to MotionMagicVoltage
     */
    private double calculateEncoderPositionForTarget(double targetX, double targetY) {
        Translation2d toTargetVector = GetTurretToHub.calculateTurretToHubVector(
                getPoseEstimatorX(),
                getPoseEstimatorY(),
                degreesToRadians(getPoseEstimatorRotation()),
                XofTurretOnBot,
                YofTurretOnBot,
                targetX,
                targetY);

        double robotRelativeAngle = normalizeAngle(
                toTargetVector.getAngle().getDegrees() - getPoseEstimatorRotation() - 180);

        return turretDegreesAndEncoderUnits(clampTurretAngle(robotRelativeAngle));
    }

    // -------------------------------------------------------------------------
    // Pose-based aiming — primary mode
    // -------------------------------------------------------------------------

    /**
     * Immediately aims the turret at the hub using the pose estimator.
     * Call this from Commands.run()-based commands for continuous tracking.
     */
    public void turretAutoAimToHubImmediate() {
        motor.setControl(new MotionMagicVoltage(
                calculateEncoderPositionForTarget(TargetXposition, TargetYposition)));
    }

    /**
     * Command: hold to continuously aim at the hub via the pose estimator.
     * This is the primary teleop aiming command — bind to a held button.
     */
    public Command aimAtHubViaPose() {
        return Commands.run(this::turretAutoAimToHubImmediate, this);
    }

    /**
     * Command: snap the turret to the hub once.
     * Used as a PathPlanner NamedCommand; for teleop tracking use aimAtHubViaPose().
     */
    public Command TurretAutoAimToHub() {
        return Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(
                calculateEncoderPositionForTarget(TargetXposition, TargetYposition))));
    }

    /**
     * Command: snap turret to current hub aim using the encoder-mapped position.
     * Bound to copilot right bumper — fires once on press.
     */
    public Command setTurretPositionVariable() {
        final double encoderPos = calculateEncoderPositionForTarget(TargetXposition, TargetYposition);
        return Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(encoderPos)));
    }

    // -------------------------------------------------------------------------
    // Lob shot — mid-field shooting toward alliance zone when hub is not lit
    // -------------------------------------------------------------------------

    /**
     * Returns the lob shot target for the current alliance.
     * Blue shoots toward the blue end zone; Red toward the red end zone.
     * Falls back to blue if alliance is not yet determined.
     */
    private double[] getLobShotTarget() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new double[] { LOB_TARGET_RED_X, LOB_TARGET_RED_Y };
        }
        return new double[] { LOB_TARGET_BLUE_X, LOB_TARGET_BLUE_Y };
    }

    /**
     * Immediately aims the turret toward the alliance lob zone.
     * The target is placed inside the alliance end zone so the ball arcs past the
     * hub and lands within the playing field. Physical turret limits ensure the
     * shot cannot be directed out of bounds.
     */
    public void lobShotImmediate() {
        double[] target = getLobShotTarget();
        motor.setControl(new MotionMagicVoltage(
                calculateEncoderPositionForTarget(target[0], target[1])));
    }

    /**
     * Command: hold to continuously aim at the alliance lob zone.
     * Use when the hub is not lit and the robot is collecting in mid-field.
     */
    public Command aimForLobShot() {
        return Commands.run(this::lobShotImmediate, this);
    }

    // -------------------------------------------------------------------------
    // Vision-based aiming (falls back to pose when hub tags not visible)
    // -------------------------------------------------------------------------

    /**
     * Command: continuously aim using Limelight txnc from the nearest hub
     * AprilTag; falls back to pose estimator when no hub tags are visible.
     */
    public Command aimAtHubViaVision() {
        return Commands.run(this::turretAimViaVisionImmediate, this);
    }

    private void turretAimViaVisionImmediate() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-three");
        RawFiducial bestTag = null;
        double closestDist = Double.MAX_VALUE;
        for (RawFiducial f : fiducials) {
            for (int id : getHubTagIDs()) {
                if (f.id == id && f.distToRobot < closestDist) {
                    bestTag = f;
                    closestDist = f.distToRobot;
                }
            }
        }

        if (bestTag != null) {
            double targetAngle = clampTurretAngle(bestTag.txnc);
            motor.setControl(new MotionMagicVoltage(turretDegreesAndEncoderUnits(targetAngle)));
        } else {
            turretAutoAimToHubImmediate();
        }
    }

    /** Returns hub AprilTag IDs for the current alliance. Red: 9, 10. Blue: 24, 25. */
    private int[] getHubTagIDs() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new int[] { 9, 10 };
        }
        return new int[] { 24, 25 };
    }

    // -------------------------------------------------------------------------
    // Direct position control
    // -------------------------------------------------------------------------

    /** Moves the turret to the center position (angle = 0°). */
    public Command TurretToZero() {
        return Commands.runOnce(() -> motor.setControl(
                new MotionMagicVoltage(turretDegreesAndEncoderUnits(0))));
    }

    /** Moves the turret to its physical maximum angle. */
    public Command TurretToMaxPosition() {
        return Commands.runOnce(() -> motor.setControl(
                new MotionMagicVoltage(turretDegreesAndEncoderUnits(TURRET_MAX_ANGLE))));
    }

    /**
     * Commands the turret to a specific angle in degrees (robot-relative).
     * The angle is clamped to physical limits before being applied.
     */
    public Command setTurretAngle(double degrees) {
        return Commands.runOnce(() -> motor.setControl(
                new MotionMagicVoltage(turretDegreesAndEncoderUnits(clampTurretAngle(degrees)))));
    }

    /** Commands the turret to a raw motor encoder position (no angle conversion). */
    public Command setTurretPosition(double position) {
        return Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(position)));
    }

    /** Imperative: directly commands the turret to a specific angle in degrees. */
    public void setTurretAngleImmediate(double degrees) {
        motor.setControl(new MotionMagicVoltage(
                turretDegreesAndEncoderUnits(clampTurretAngle(degrees))));
    }

    // -------------------------------------------------------------------------
    // Stop
    // -------------------------------------------------------------------------

    public Command TurretTestStop() {
        return Commands.runOnce(() -> motor.set(0));
    }

    public void turretTestStopImmediate() {
        motor.set(0);
    }

    // -------------------------------------------------------------------------
    // Utility / telemetry
    // -------------------------------------------------------------------------

    /**
     * Returns the straight-line distance from the turret to the hub in meters,
     * accounting for the turret's actual field position.
     */
    public double getDistanceToHub() {
        return GetTurretToHub.calculateTurretToHubVector(
                getPoseEstimatorX(),
                getPoseEstimatorY(),
                degreesToRadians(getPoseEstimatorRotation()),
                XofTurretOnBot,
                YofTurretOnBot,
                TargetXposition,
                TargetYposition).getNorm();
    }

    public Translation2d getCameraPositionOnBot() {
        return new Translation2d(XofCameraOnBot, YofCameraOnBot);
    }

    public Translation2d getTurretPositionOnBot() {
        return new Translation2d(XofTurretOnBot, YofTurretOnBot);
    }

    /** Publishes aiming diagnostics to SmartDashboard for tuning. Runs at 2 Hz. */
    private void updateDashboard() {
        Translation2d toHubVector = GetTurretToHub.calculateTurretToHubVector(
                getPoseEstimatorX(),
                getPoseEstimatorY(),
                degreesToRadians(getPoseEstimatorRotation()),
                XofTurretOnBot,
                YofTurretOnBot,
                TargetXposition,
                TargetYposition);

        double targetAngle = clampTurretAngle(
                normalizeAngle(toHubVector.getAngle().getDegrees() - getPoseEstimatorRotation() - 180));

        SmartDashboard.putNumber("Turret Hub Target Angle (deg)", targetAngle);
        SmartDashboard.putNumber("Turret Hub Target Encoder", turretDegreesAndEncoderUnits(targetAngle));
        SmartDashboard.putNumber("Turret Distance to Hub (m)", getDistanceToHub());
    }
}
