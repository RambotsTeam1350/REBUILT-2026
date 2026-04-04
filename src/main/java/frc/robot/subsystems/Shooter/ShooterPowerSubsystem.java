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
        private final TalonFX backspinMotor;

        public ShooterPowerSubsystem() {
           motor1 = new TalonFX(40);
           motor2 = new TalonFX(41);
           backspinMotor = new TalonFX(42);

           TalonFXConfiguration cfg = new TalonFXConfiguration();
           cfg.CurrentLimits = new CurrentLimitsConfigs()
               .withStatorCurrentLimit(60)
               .withStatorCurrentLimitEnable(true)
               .withSupplyCurrentLimit(40)
               .withSupplyCurrentLimitEnable(true);
           motor1.getConfigurator().apply(cfg);
           motor2.getConfigurator().apply(cfg);
           backspinMotor.getConfigurator().apply(cfg);
        }
    
        public void runMotor1(double speed) {
            motor1.set(speed);
        }

        public void runMotor2(double speed) {
            motor2.set(speed);
        }
    
        public void runBackspinMotor(double speed) {
            backspinMotor.set(speed);
        }

        public void stopMotor() {
            motor1.set(0);
            motor2.set(0);
            backspinMotor.set(0);
        }

        public Command runMotorCommand() {
            return Commands.runOnce(
            () -> {
                runMotor1(1);
                runMotor2(-1);
                runBackspinMotor(-1);
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

