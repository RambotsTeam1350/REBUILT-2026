package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command that aligns the turret to the hub using AprilTag detection from the
 * Limelight.
 * Automatically detects alliance color and looks for the correct hub AprilTags:
 * - Red Alliance: AprilTags 9 and 10
 * - Blue Alliance: AprilTags 24 and 25
 *
 * Uses vision data to calculate the angle from the turret to the hub and
 * rotates the turret accordingly.
 * Accounts for the geometric offset between the camera and turret positions on
 * the robot.
 */
public class AlignToHub extends Command {
    private final TurretSubsystem turretSubsystem;
    private final String limelightName = "limelight-three";

    // Hub AprilTag IDs by alliance
    private static final int[] RED_HUB_TAGS = { 9, 10 };
    private static final int[] BLUE_HUB_TAGS = { 24, 25 };

    private static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;
    private int consecutiveOnTargetCount = 0;
    private static final int REQUIRED_ON_TARGET_COUNT = 5;

    /**
     * Creates a new AlignToHub command.
     * Automatically detects alliance color and targets the appropriate hub
     * AprilTags.
     *
     * @param turretSubsystem The turret subsystem to control
     */
    public AlignToHub(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    /**
     * Gets the AprilTag IDs for the current alliance's hub.
     *
     * @return Array of AprilTag IDs for the hub (Red: 9,10 or Blue: 24,25)
     */
    private int[] getHubTagIDs() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return RED_HUB_TAGS;
        } else {
            // Default to Blue if alliance not available or is Blue
            return BLUE_HUB_TAGS;
        }
    }

    /**
     * Checks if the given AprilTag ID is a hub tag for the current alliance.
     *
     * @param tagID The AprilTag ID to check
     * @return true if this tag is a hub tag for the current alliance
     */
    private boolean isHubTag(int tagID) {
        int[] hubTags = getHubTagIDs();
        for (int hubTag : hubTags) {
            if (tagID == hubTag) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void initialize() {
        consecutiveOnTargetCount = 0;
    }

    @Override
    public void execute() {
        // Get raw fiducial data from Limelight
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);

        // Find a hub AprilTag (any of the alliance-specific tags)
        // Prefer the closest one if multiple are visible
        RawFiducial hubTag = null;
        double closestDistance = Double.MAX_VALUE;

        for (RawFiducial fiducial : fiducials) {
            if (isHubTag(fiducial.id)) {
                // Use the closest hub tag if multiple are visible
                if (fiducial.distToRobot < closestDistance) {
                    hubTag = fiducial;
                    closestDistance = fiducial.distToRobot;
                }
            }
        }

        if (hubTag != null) {
            // txnc is the horizontal angle to the target in degrees (not normalized).
            // Positive = target is to the right of the crosshair, negative = left.
            double angleToTagFromCamera = hubTag.txnc;

            // Get current turret angle
            double currentTurretAngle = turretSubsystem.getTurretRotation();

            // Calculate the geometric offset between camera and turret positions
            Translation2d cameraPos = turretSubsystem.getCameraPositionOnBot();
            Translation2d turretPos = turretSubsystem.getTurretPositionOnBot();
            Translation2d cameraToTurretOffset = turretPos.minus(cameraPos);

            // Calculate the angular offset caused by the camera-turret position difference
            // This is the angle from the camera's line-of-sight to the turret's
            // line-of-sight
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

            // Command the turret to the target angle (imperative call so it runs now)
            turretSubsystem.setTurretAngleImmediate(targetTurretAngle);

            // Check if we're on target
            if (Math.abs(totalAngleCorrection) < ALIGNMENT_TOLERANCE_DEGREES) {
                consecutiveOnTargetCount++;
            } else {
                consecutiveOnTargetCount = 0;
            }
        } else {
            // If we can't see the tag, use pose estimator fallback (imperative)
            turretSubsystem.turretAutoAimToHubImmediate();
            consecutiveOnTargetCount = 0;
        }
        System.out.println("Command Ran");
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
            turretSubsystem.turretTestStopImmediate();
        }
    }
}
