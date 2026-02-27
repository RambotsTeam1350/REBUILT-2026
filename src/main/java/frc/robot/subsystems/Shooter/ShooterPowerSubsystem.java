package frc.robot.subsystems.Shooter;

import com.revrobotics.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterPowerSubsystem {
    
        private final SparkFlex motor = new SparkFlex(16, MotorType.kBrushless);
        private final RelativeEncoder encoder = motor.getEncoder();
    
        public ShooterPowerSubsystem() {
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
                runMotor(10);


                
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
