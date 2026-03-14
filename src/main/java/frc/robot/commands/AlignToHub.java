package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command that aligns the turret to the hub using AprilTag detection from the Limelight.
 * Uses vision data to calculate the angle from the turret to the hub and rotates the turret accordingly.
 */
public class AlignToHub extends Command {
    private final TurretSubsystem turretSubsystem;
    private final String limelightName = "limelight-fifteen";
    private final int hubAprilTagID;

    private static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;
    private int consecutiveOnTargetCount = 0;
    private static final int REQUIRED_ON_TARGET_COUNT = 5;

    /**
     * Creates a new AlignToHub command.
     *
     * @param turretSubsystem The turret subsystem to control
     * @param hubAprilTagID The AprilTag ID of the hub to align to
     */
    public AlignToHub(TurretSubsystem turretSubsystem, int hubAprilTagID) {
        this.turretSubsystem = turretSubsystem;
        this.hubAprilTagID = hubAprilTagID;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        consecutiveOnTargetCount = 0;
    }

    @Override
    public void execute() {
        // Get raw fiducial data from Limelight
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);

        // Find the hub AprilTag
        RawFiducial hubTag = null;
        for (RawFiducial fiducial : fiducials) {
            if (fiducial.id == hubAprilTagID) {
                hubTag = fiducial;
                break;
            }
        }

        if (hubTag != null) {
            // txnc is the normalized horizontal offset from crosshair (-1 to 1)
            // Positive txnc means target is to the right, negative means left
            double horizontalAngleOffset = hubTag.txnc;

            // Get current turret angle
            double currentTurretAngle = turretSubsystem.getTurretRotation();

            // Calculate the target angle for the turret
            // txnc is normalized (-1 to 1), so we need to scale it to the camera's field of view
            // Limelight has approximately 63.3 degree horizontal FOV
            double cameraHorizontalFOV = 63.3;
            double angleToTagDegrees = horizontalAngleOffset * (cameraHorizontalFOV / 2.0);

            // The target angle is the current angle plus the offset to the tag
            double targetTurretAngle = currentTurretAngle + angleToTagDegrees;

            // Command the turret to the target angle
            turretSubsystem.setTurretAngle(targetTurretAngle);

            // Check if we're on target
            if (Math.abs(angleToTagDegrees) < ALIGNMENT_TOLERANCE_DEGREES) {
                consecutiveOnTargetCount++;
            } else {
                consecutiveOnTargetCount = 0;
            }
        } else {
            // If we can't see the tag, use pose estimator fallback
            turretSubsystem.TurretAutoAimToHub();
            consecutiveOnTargetCount = 0;
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes when we've been on target for the required number of cycles
        return consecutiveOnTargetCount >= REQUIRED_ON_TARGET_COUNT;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Stop the turret if interrupted
            turretSubsystem.TurretTestStop();
        }
    }
}
