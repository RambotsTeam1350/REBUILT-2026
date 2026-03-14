package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command that aligns the turret to the hub using AprilTag detection from the Limelight.
 * Uses vision data to calculate the angle from the turret to the hub and rotates the turret accordingly.
 * Accounts for the geometric offset between the camera and turret positions on the robot.
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

            // Calculate the angle to the tag from the camera's perspective
            // txnc is normalized (-1 to 1), so we need to scale it to the camera's field of view
            // Limelight has approximately 63.3 degree horizontal FOV
            double cameraHorizontalFOV = 63.3;
            double angleToTagFromCamera = horizontalAngleOffset * (cameraHorizontalFOV / 2.0);

            // Calculate the geometric offset between camera and turret positions
            Translation2d cameraPos = turretSubsystem.getCameraPositionOnBot();
            Translation2d turretPos = turretSubsystem.getTurretPositionOnBot();
            Translation2d cameraToTurretOffset = turretPos.minus(cameraPos);

            // Calculate the angular offset caused by the camera-turret position difference
            // This is the angle from the camera's line-of-sight to the turret's line-of-sight
            // at the target distance
            double distanceToTarget = hubTag.distToRobot; // in meters
            double geometricAngleOffset = 0.0;

            if (distanceToTarget > 0.1) { // Avoid division by very small numbers
                // Use small angle approximation: angle ≈ perpendicular offset / distance
                // The perpendicular offset depends on the camera viewing angle
                double angleToTagRadians = Math.toRadians(angleToTagFromCamera);
                double perpendicularOffset = cameraToTurretOffset.getX() * Math.sin(angleToTagRadians)
                                            - cameraToTurretOffset.getY() * Math.cos(angleToTagRadians);
                geometricAngleOffset = Math.toDegrees(Math.atan2(perpendicularOffset, distanceToTarget));
            }

            // The total angle correction is the camera angle plus the geometric offset
            double totalAngleCorrection = angleToTagFromCamera + geometricAngleOffset;

            // The target turret angle is the current angle plus the correction
            double targetTurretAngle = currentTurretAngle + totalAngleCorrection;

            // Command the turret to the target angle
            turretSubsystem.setTurretAngle(targetTurretAngle);

            // Check if we're on target
            if (Math.abs(totalAngleCorrection) < ALIGNMENT_TOLERANCE_DEGREES) {
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
