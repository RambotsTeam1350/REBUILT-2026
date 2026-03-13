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
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Barcode;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
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

    public double TargetXposition = 4.625594; //Xₜ 182.11 in inches
    public double TargetYposition = 4.03479; //Yₜ 158.85 in inches 
    public double TargetRotation;

    public double XofTurretOnBot = 0.05; //XofTurretOnBot
    public double YofTurretOnBot = -0.05;

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

        GetTurretToHub.calculateTurretToHubVector(getPoseEstimatorX(), getPoseEstimatorY(), degreesToRadians(getPoseEstimatorRotation()), XofTurretOnBot, YofTurretOnBot, TargetXposition, TargetYposition);
        //System.out.println(GetTurretToHub.calculateTurretToHubVector(getPoseEstimatorX(), getPoseEstimatorY(), degreesToRadians(getPoseEstimatorRotation()), XofTurretOnBot, YofTurretOnBot, TargetXposition, TargetYposition).getAngle().getDegrees());
        
        TurretAutoAimToHub();
        //System.out.println(getAngleToTarget());

        // ^ make sure all of these numbers are being updated frequently so the turret is always aiming at the right place
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

public double getTargetRotation()
{
    return Math.atan2((TargetYposition - getPoseEstimatorY()), (TargetXposition - getPoseEstimatorX()));
}
//TargetRotation - poseEstimator.getEstimatedPosition().getRotation().getDegrees(); //θᵣₑₗₐₜᵢᵥₑ
public double getAngleToTarget() {
    return (getPoseEstimatorRotation() - getTargetRotation());
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

    public Command TurretAutoAimToHub() {
        return Commands.sequence(
                Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(
                GetTurretToHub.calculateTurretToHubVector(getPoseEstimatorX(), getPoseEstimatorY(), degreesToRadians(getPoseEstimatorRotation()), XofTurretOnBot, YofTurretOnBot, TargetXposition, TargetYposition).getAngle().getDegrees()
                )))));
       }
     

    public Command TurretToZero() {
        return Commands.sequence(
                Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(0)))
            );
       } 
}