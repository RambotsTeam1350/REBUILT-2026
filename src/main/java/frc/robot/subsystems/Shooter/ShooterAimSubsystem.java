package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShooterAimSubsystem extends SubsystemBase {
    
    public double heightOfHub = 2.64; // in meters
    public double heightOfBot = 0.4495; // in meters

    public double TargetXposition = 4.625594; //Xₜ 182.11 in inches
    public double TargetYposition = 4.03479; //Yₜ 158.85 in inches 

    public double heightForAiming = heightOfHub - heightOfBot; // in meters
    private SwerveDrivePoseEstimator poseEstimator;
    private final TalonFX motor;
    private StatusSignal<Angle> motorPosition;
    private double gearBoxRatio = 9.0; // Assuming a 9:1 gear ratio for the shooter angle adjuster

     public ShooterAimSubsystem(SwerveDrivePoseEstimator poseEstimator) {
         this.poseEstimator = poseEstimator;
        // constructor code here, if needed
        motor = new TalonFX(14);


         TalonFXConfiguration cfg = new TalonFXConfiguration();
            cfg.Slot0.kP = 4.8;
            cfg.Slot0.kI = 0;
            cfg.Slot0.kD = 0.1;
            cfg.Slot0.kV = .12;
            cfg.Slot0.kA = .01;
            cfg.Slot0.kS = .25;

            MotionMagicConfigs mm = cfg.MotionMagic;
            mm.MotionMagicCruiseVelocity = 6; 
            mm.MotionMagicAcceleration = 80;
            mm.MotionMagicJerk = 1600; 
            motor.getConfigurator().apply(cfg);

            this.motorPosition = this.motor.getPosition();
    }

public void periodic() {

        getPoseEstimatorX();
        getPoseEstimatorY();
        getPoseEstimatorRotation();

        // ^ make sure all of these numbers are being updated frequently so the turret is always aiming at the right place
        BaseStatusSignal.refreshAll(motorPosition);
        System.out.print(motor.getPosition().getValueAsDouble() + " Turret Motor Position");
        
    }

    public double getPoseEstimatorY() {
    return poseEstimator.getEstimatedPosition().getY();
}

public double getPoseEstimatorX() {
    return poseEstimator.getEstimatedPosition().getX();
}

public double getPoseEstimatorRotation() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
}

public double getAngleAdjusterTheta() {
    double deltaX = TargetXposition - getPoseEstimatorX();
    double deltaY = TargetYposition - getPoseEstimatorY();
    return Math.toDegrees(Math.atan2(heightForAiming, Math.sqrt(deltaX * deltaX + deltaY * deltaY)));
}

private double degreesToEncoderUnits(double degrees) {
        // Assuming 2048 units per revolution and a gear ratio of 9:1
        double unitsPerRevolution = 2048;
        return (degrees / 360.0) * unitsPerRevolution * gearBoxRatio;
    }

    public Command AngleAdjust() {
        return Commands.sequence(
            Commands.runOnce(() ->  motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(getAngleAdjusterTheta()))))
        );
    }

}
