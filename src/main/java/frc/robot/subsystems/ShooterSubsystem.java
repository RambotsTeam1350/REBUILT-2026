package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterSubsystem {
    
        private final SparkFlex motor = new SparkFlex(16, MotorType.kBrushless);
        private final RelativeEncoder encoder = motor.getEncoder();
    
        public ShooterSubsystem() {
           motor.setInverted(true);
        }
    
        public void runMotor(double speed) {
            motor.set(speed);
        }
    
        public void stopMotor() {
            motor.set(0); 
        }
    
        public double getVelocity() {
            return encoder.getVelocity();
        }

        public Command runMotorCommand() {
            return Commands.runOnce(
            () -> {
                runMotor(0.3);
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
