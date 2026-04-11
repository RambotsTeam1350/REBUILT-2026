package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;

public class GetTurretToHub extends Command {

    /**
     * Calculates the vector from the robot's turret to a field hub.
     * NOTE: This returns a field-absolute vector. For turret control, use
     * calculateRobotRelativeAngleToHub().
     *
     * @param robotX            Robot center X coordinate (field frame). // pose
     *                          estimator
     * @param robotY            Robot center Y coordinate (field frame). // pose
     *                          estimator
     * @param robotThetaRadians Robot orientation relative to field axes. // pose
     *                          estimator
     * @param turretRelX        Turret X position relative to robot center (robot
     *                          frame). // constant
     * @param turretRelY        Turret Y position relative to robot center (robot
     *                          frame). // constant - both of these from the center
     *                          of the bot and relative to it
     * @param hubX              Hub X coordinate (field frame). // constant
     * @param hubY              Hub Y coordinate (field frame). // constant
     * @return Translation2d representing the vector from the turret to the hub in
     *         the field frame.
     */

    public static Translation2d calculateTurretToHubVector(
            double robotX,
            double robotY,
            double robotThetaRadians,
            double turretRelX,
            double turretRelY,
            double hubX,
            double hubY) {

        // Conditional logging flag
        boolean turretAimLogging = false;

        // 1. Define the robot's pose in the global field coordinate system
        Pose2d robotPose = new Pose2d(robotX, robotY, new Rotation2d(robotThetaRadians));

        // 2. Define the turret's position relative to the robot center.
        // We use Transform2d to represent this local offset.
        Transform2d turretTransform = new Transform2d(
                new Translation2d(turretRelX, turretRelY),
                new Rotation2d(0) // We only care about translation here
        );

        // 3. Calculate the turret's absolute pose in the field coordinate system.
        Pose2d turretFieldPose = robotPose.transformBy(turretTransform);
        Translation2d turretFieldTranslation = turretFieldPose.getTranslation();

        // 4. Define the hub's position in the field coordinate system
        Translation2d hubTranslation = new Translation2d(hubX, hubY);

        // 5. Calculate the vector from the turret to the hub (Destination - Source)
        // In Java, we use the .minus() method since there is no operator overloading
        Translation2d turretToHubVector = hubTranslation.minus(turretFieldTranslation);

        // 6. Log the parameters and result if the flag is true
        if (turretAimLogging) {
            // FileWriter(filename, true) enables append mode. It creates the file if
            // missing.
            try (PrintWriter out = new PrintWriter(new FileWriter("TurretAiming.log", true))) {
                out.printf("[%s] Inputs: Robot(%.3f, %.3f, %.3f rad), TurretRel(%.3f, %.3f), Hub(%.3f, %.3f) | " +
                        "Output Vector: X=%.3f, Y=%.3f%n",
                        LocalDateTime.now(), // Added a timestamp to make the log easier to read
                        robotX, robotY, robotThetaRadians,
                        turretRelX, turretRelY,
                        hubX, hubY,
                        turretToHubVector.getX(), turretToHubVector.getY());
            } catch (IOException e) {
                System.err.println("Failed to write to TurretAiming.log: " + e.getMessage());
            }
        }

        return turretToHubVector;
    }

    /**
     * Calculates the robot-relative angle from the turret to the hub.
     * This is the correct method to use for turret aiming commands.
     *
     * @param robotX            Robot center X coordinate (field frame)
     * @param robotY            Robot center Y coordinate (field frame)
     * @param robotThetaRadians Robot orientation relative to field axes
     * @param turretRelX        Turret X position relative to robot center (robot
     *                          frame)
     * @param turretRelY        Turret Y position relative to robot center (robot
     *                          frame)
     * @param hubX              Hub X coordinate (field frame)
     * @param hubY              Hub Y coordinate (field frame)
     * @return Robot-relative angle in degrees (normalized to [-180, 180])
     */
    public static double calculateRobotRelativeAngleToHub(
            double robotX,
            double robotY,
            double robotThetaRadians,
            double turretRelX,
            double turretRelY,
            double hubX,
            double hubY) {

        // Get the field-absolute vector
        Translation2d turretToHubVector = calculateTurretToHubVector(
                robotX, robotY, robotThetaRadians,
                turretRelX, turretRelY,
                hubX, hubY);

        // Get field-absolute angle
        double fieldAngleDegrees = turretToHubVector.getAngle().getDegrees();

        // Convert robot orientation to degrees
        double robotThetaDegrees = Math.toDegrees(robotThetaRadians);

        // Convert to robot-relative angle
        double robotRelativeAngle = fieldAngleDegrees - robotThetaDegrees;

        // Normalize to [-180, 180]
        while (robotRelativeAngle > 180)
            robotRelativeAngle -= 360;
        while (robotRelativeAngle < -180)
            robotRelativeAngle += 360;

        return robotRelativeAngle;
    }

    // --- Example Usage ---
    public static void main(String[] args) {
        // Example: Robot is at (2.0, 3.0) meters on the field, facing 90 degrees (pi/2
        // radians)
        // The turret is mounted 0.5 meters forward of the robot center (X offset).
        // The hub is located at (8.0, 4.0) on the field.

        Translation2d vectorToHub = calculateTurretToHubVector(
                2.0, 3.0, Math.PI / 2.0, 0.5, 0.0, 8.0, 4.0);

        System.out.printf("Vector to hub (Field Frame): X = %.3f, Y = %.3f%n", vectorToHub.getX(), vectorToHub.getY());
        System.out.printf("Distance to hub: %.3f meters%n", vectorToHub.getNorm());
        System.out.printf("Global Angle to hub: %.3f degrees%n", vectorToHub.getAngle().getDegrees());
    }
}