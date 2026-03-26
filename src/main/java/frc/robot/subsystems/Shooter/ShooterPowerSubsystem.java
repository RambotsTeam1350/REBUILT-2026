package frc.robot.subsystems.Shooter;

import com.revrobotics.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPowerSubsystem extends SubsystemBase{
    
        private final TalonFX motor1;
        private final TalonFX motor2;

        public ShooterPowerSubsystem() {
           motor1 = new TalonFX(40);
           motor2 = new TalonFX(41);

           TalonFXConfiguration cfg = new TalonFXConfiguration();
           cfg.CurrentLimits = new CurrentLimitsConfigs()
               .withStatorCurrentLimit(60)
               .withStatorCurrentLimitEnable(true)
               .withSupplyCurrentLimit(40)
               .withSupplyCurrentLimitEnable(true);
           motor1.getConfigurator().apply(cfg);
           motor2.getConfigurator().apply(cfg);
        }
    
        public void runMotor1(double speed) {
            motor1.set(speed);
        }

        public void runMotor2(double speed) {
            motor2.set(speed);
        }
    
        public void stopMotor() {
            motor1.set(0);
            motor2.set(0);
        }

        public Command runMotorCommand() {
            return Commands.runOnce(
            () -> {
                runMotor1(1);
                runMotor2(-1);
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
}

