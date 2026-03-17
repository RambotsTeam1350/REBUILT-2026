package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Vector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.GetTurretToHub;

/*  
    This subsystem will eventaully use auto-aiming features and kinematics from the pose estimator, 
    but right now for testing purposes it is only a motor running
*/
public class TurretSubsystem extends SubsystemBase {
    

    private final TalonFX motor;
    //public double PoseEstimatorXposition = CommandSwerveDrivetrain.positiveXDistance; //Xᵣ
    //public double PoseEstimatorYposition = 0; //Yᵣ
    //public double PoseEstimatorRotation = 0; //θᵣ

    double gearBoxRatio = 3.0; // Assuming a 9:1 gear ratio for the turret
    private StatusSignal<Angle> motorPosition;

    // Turret physical limits (degrees relative to robot front)
    // The turret can rotate approximately 180 degrees, centered on the back of the robot
    private static final double TURRET_MIN_ANGLE = -90.0; // 90 degrees left of back = right side
    private static final double TURRET_MAX_ANGLE = 90.0;  // 90 degrees right of back = left side

    public double TargetXposition = 4.625594; //Xₜ 182.11 in inches
    public double TargetYposition = 4.03479; //Yₜ 158.85 in inches 
    public double TargetRotation;

    public double XofTurretOnBot = -0.05; //XofTurretOnBot
    public double YofTurretOnBot = -0.05;

    // Limelight camera position relative to robot center (in meters)
    public double XofCameraOnBot = 0.0; // TODO: Measure actual camera X offset
    public double YofCameraOnBot = 0.0; // TODO: Measure actual camera Y offset

    public double AngleToTarget;

    private SwerveDrivePoseEstimator poseEstimator;

    public TurretSubsystem(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
        motor = new TalonFX(13);


         TalonFXConfiguration cfg = new TalonFXConfiguration();
            cfg.Slot0.kP = 4.8;
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

            this.motorPosition = this.motor.getPosition();

    }

    public double degreesToRadians(double degrees) {
        return degrees * (Math.PI / 180);
    }

@Override
    public void periodic() {

        getPoseEstimatorX();
        getPoseEstimatorY();
        getPoseEstimatorRotation();
        getTargetRotation();
        getAngleToTarget();

        // Continuously command the motor to track the hub every loop.
        // turretAutoAimToHubImmediate() drives the motor directly; TurretAutoAimToHub()
        // only returns a Command object and must not be called here (it was a no-op).
        turretAutoAimToHubImmediate();

        BaseStatusSignal.refreshAll(motorPosition);
        //System.out.print(motor.getPosition().getValueAsDouble() + " Turret Motor Position");
        
    }
 
/* ------------------------------------------------------------------------------ */
public double getPoseEstimatorY() {
    return poseEstimator.getEstimatedPosition().getY();
}

public double getPoseEstimatorX() {
    return poseEstimator.getEstimatedPosition().getX();
}

public double getPoseEstimatorRotation() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
}

public double getTurretRotation() {
    return encoderUnitsToDegrees(motor.getPosition().getValueAsDouble());
}
/* ------------------------------------------------------------------------------ */

// Returns the field-absolute angle from the robot to the target, in degrees.
public double getTargetRotation()
{
    return Math.toDegrees(Math.atan2((TargetYposition - getPoseEstimatorY()), (TargetXposition - getPoseEstimatorX())));
}

// Returns the robot-relative angle to the target, in degrees.
public double getAngleToTarget() {
    return normalizeAngle(getTargetRotation() - getPoseEstimatorRotation());
}
/* --------- v new calculations as of 3/7/26 v --------- */

public Translation2d getDistanceBotonTurretFieldRelative() { //step 1 of the five step plan :)
    return new Translation2d(XofTurretOnBot * Math.cos(degreesToRadians(getPoseEstimatorRotation())), 
    YofTurretOnBot * Math.sin(degreesToRadians(getPoseEstimatorRotation())));
}

public Translation2d getPositionTurretonField() {
    return new Translation2d();
}

public double getTurretAngleBotRelative() {
    return 3 - getPoseEstimatorRotation();
}

/* --------- ^ new calculations as of 3/7/26 ^ --------- */

private double degreesToEncoderUnits(double degrees) {
        // Assuming 2048 units per revolution and a gear ratio of 9:1
        double unitsPerRevolution = 2048;
        return ((degrees / 360.0) * (8.5810546875/9)) * gearBoxRatio; // 8.58 is what full revolution actualy is, instead of 9
    }

private double encoderUnitsToDegrees(double encoderUnits) {
    return ( 360 * ( encoderUnits / ((8.5810546875/9) * gearBoxRatio)));
}

/**
 * Normalizes an angle to the range [-180, 180] degrees.
 *
 * @param degrees The angle to normalize
 * @return The normalized angle in [-180, 180] range
 */
private double normalizeAngle(double degrees) {
    double angle = degrees;
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

/**
 * Clamps a turret angle to the physical limits of the turret mechanism.
 *
 * @param degrees The desired turret angle
 * @return The clamped angle within [TURRET_MIN_ANGLE, TURRET_MAX_ANGLE]
 */
private double clampTurretAngle(double degrees) {
    return Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, degrees));
}
    public Command TurretToMaxPosition() {
        return Commands.sequence(
                Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(360))))
            );
       } 
/* 
       public Command TurretTestSpeed() {
        return Commands.sequence(
                Commands.runOnce(() -> motor.set(0.1))
            );
       }
*/
   
       //_________________________________________________________________________________________________

       public Command TurretTestStop() {
        return Commands.sequence(
                Commands.runOnce(() -> motor.set(0))
            );
       }

    /**
     * Imperative immediate stop for the turret motor.
     */
    public void turretTestStopImmediate() {
        motor.set(0);
    }

    public Command TurretAutoAimToHub() {
        return Commands.runOnce(() -> {
            // Calculate vector from turret to hub in field frame
            Translation2d turretToHubVector = GetTurretToHub.calculateTurretToHubVector(
                getPoseEstimatorX(),
                getPoseEstimatorY(),
                degreesToRadians(getPoseEstimatorRotation()),
                XofTurretOnBot,
                YofTurretOnBot,
                TargetXposition,
                TargetYposition
            );

            // Get field-absolute angle to hub
            double fieldAngleToHub = turretToHubVector.getAngle().getDegrees();

            // Convert to robot-relative angle
            // Field angle - robot rotation = robot-relative angle
            double robotRelativeAngle = fieldAngleToHub - getPoseEstimatorRotation();

            // Normalize to [-180, 180] range
            robotRelativeAngle = normalizeAngle(robotRelativeAngle);

            // Clamp to turret physical limits
            robotRelativeAngle = clampTurretAngle(robotRelativeAngle);

            // Command turret to robot-relative angle
            motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(robotRelativeAngle)));
        });
    }
     

    public Command TurretToZero() {
        return Commands.sequence(
                Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(0)))
            );
       }

    /**
     * Sets the turret to a specific angle in degrees using MotionMagic control.
     *
     * @param degrees The target angle in degrees
     */
    public Command setTurretAngle(double degrees) {
        return Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(degrees))));
    }

    /**
     * Immediately set the turret to a specific angle (imperative API).
     * Use this from other commands when you want to directly command the motor
     * without creating/scheduling a Command object.
     *
     * @param degrees The target angle in degrees
     */
    public void setTurretAngleImmediate(double degrees) {
        motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(degrees)));
    }

    /**
     * Returns hub AprilTag IDs for the current alliance.
     * Red: tags 9, 10 — Blue: tags 24, 25.
     */
    private int[] getHubTagIDs() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new int[]{9, 10};
        }
        return new int[]{24, 25};
    }

    /**
     * Aims the turret at the hub using a hybrid strategy:
     *
     *   1. Direct vision (primary): when a hub AprilTag is visible in limelight-three,
     *      txnc (horizontal angle in degrees) is used directly. This skips the pose
     *      estimation chain entirely and gives ~0.1° accuracy vs ~1-3° for pose-based.
     *
     *   2. Pose estimator fallback: when no hub tags are visible, the field-position
     *      vector calculation is used so the turret keeps tracking while driving.
     *
     * NOTE: Set XofCameraOnBot / YofCameraOnBot to the measured camera position
     * (meters from robot center) to make the geometric correction accurate.
     */
    public void turretAutoAimToHubImmediate() {
        // --- Try direct vision first ---
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
            // txnc is the horizontal angle to the tag in degrees (positive = right of crosshair).
            // Apply geometric correction for the offset between camera and turret positions.
            double angleToTagDeg = bestTag.txnc;
            double geometricOffsetDeg = 0.0;
            if (bestTag.distToRobot > 0.1) {
                Translation2d camOffset = new Translation2d(
                    XofCameraOnBot - XofTurretOnBot,
                    YofCameraOnBot - YofTurretOnBot);
                double angleRad = Math.toRadians(angleToTagDeg);
                double perpOffset = camOffset.getX() * Math.sin(angleRad)
                                  - camOffset.getY() * Math.cos(angleRad);
                geometricOffsetDeg = Math.toDegrees(Math.atan2(perpOffset, bestTag.distToRobot));
            }
            double targetAngle = clampTurretAngle(
                getTurretRotation() + angleToTagDeg + geometricOffsetDeg);
            motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(targetAngle)));

        } else {
            // --- Pose estimator fallback ---
            Translation2d turretToHubVector = GetTurretToHub.calculateTurretToHubVector(
                getPoseEstimatorX(),
                getPoseEstimatorY(),
                degreesToRadians(getPoseEstimatorRotation()),
                XofTurretOnBot,
                YofTurretOnBot,
                TargetXposition,
                TargetYposition
            );
            double robotRelativeAngle = normalizeAngle(
                turretToHubVector.getAngle().getDegrees() - getPoseEstimatorRotation());
            robotRelativeAngle = clampTurretAngle(robotRelativeAngle);
            motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(robotRelativeAngle)));
        }
    }

    /**
     * Returns the straight-line distance from the turret to the hub in meters,
     * accounting for the turret's actual field position (robot pose + heading + offset).
     * Use this as the single authoritative distance for both turret and shooter angle.
     */
    public double getDistanceToHub() {
        return GetTurretToHub.calculateTurretToHubVector(
            getPoseEstimatorX(),
            getPoseEstimatorY(),
            degreesToRadians(getPoseEstimatorRotation()),
            XofTurretOnBot,
            YofTurretOnBot,
            TargetXposition,
            TargetYposition
        ).getNorm();
    }

    /**
     * Gets the camera position relative to robot center.
     *
     * @return Translation2d representing camera position in robot frame
     */
    public Translation2d getCameraPositionOnBot() {
        return new Translation2d(XofCameraOnBot, YofCameraOnBot);
    }

    /**
     * Gets the turret position relative to robot center.
     *
     * @return Translation2d representing turret position in robot frame
     */
    public Translation2d getTurretPositionOnBot() {
        return new Translation2d(XofTurretOnBot, YofTurretOnBot);
    }
}