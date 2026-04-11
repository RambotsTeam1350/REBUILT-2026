package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDCANdleSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class FieldAreaCheckerSubsystem extends SubsystemBase {

    private final LEDCANdleSubsystem ledSubsystem = new LEDCANdleSubsystem();
    private final double minX = 2.0; // left boundary example values, adjust as needed
    private final double maxX = 4.0; // right boundary example values, adjust as needed
    private final double minY = 1.0; // bottom boundary example values, adjust as needed
    private final double maxY = 3.0; // top boundary example values, adjust as needed
    private final double minA = -90.0;
    private final double maxA = 90.0;

    public boolean isRobotInArea(Pose2d robotpose) {
        double x = robotpose.getX();
        double y = robotpose.getY();
        double a = robotpose.getRotation().getDegrees();
        return (x >= minX && x <= maxX) && (y >= minY && y <= maxY) && (a >= minA && a <= maxA);
    }

    // need to make this method repeat itself, so that the LED will update as the
    // robot moves in and out of the area
    public Command checkFieldArea(Pose2d robotpose) {
        if (isRobotInArea(robotpose)) {
            System.out.println("Robot is in the desired area, good to align and shoot.");
            return Commands.runOnce(() -> ledSubsystem.LEDGreen());
        } else {
            System.out.println("Robot is outside the desired area, MOVE!");
            return Commands.runOnce(() -> ledSubsystem.LEDRed());
        }
    }
}