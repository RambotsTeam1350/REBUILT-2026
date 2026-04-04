package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWheelSubsystem extends SubsystemBase {
    
    private final TalonFX motor;

    public final StatusSignal<AngularVelocity> velocity;
    public final StatusSignal<Angle> position;

    public double velocityDouble = 0.0;
    public static double positionDouble = 0.0;

    public IntakeWheelSubsystem() {
        motor = new TalonFX(34);

        velocity = motor.getVelocity();
        position = motor.getPosition();

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLimitEnable(true);
        motor.getConfigurator().apply(cfg);
    }

     public void runMotor(double speed) {
        motor.set(speed);
    }
 
     public void stopMotor() {
        motor.set(0); 

    }

    public void reverseMotor(double speed) {
        motor.set(speed);
    }
 
      
        public Command runMotorCommand() {
        return Commands.runOnce(
            () -> {
                runMotor(0.5);
            }
        );
    }

        public Command runMotorCommand(double speed) {
        return Commands.runOnce(
            () -> {
                runMotor(speed);
            }
        );
    }
 
     public Command stopMotorCommand() {
        return Commands.runOnce(
            () -> {
                stopMotor();
            }
        );
      }

      public Command reverseMotorCommand() {
        return Commands.runOnce(
            () -> {
                reverseMotor(.7);
            }
        );
      }
}
