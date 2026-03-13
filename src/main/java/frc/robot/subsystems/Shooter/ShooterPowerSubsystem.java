package frc.robot.subsystems.Shooter;

import com.revrobotics.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterPowerSubsystem {
    
        private final SparkFlex motor1 = new SparkFlex(16, MotorType.kBrushless);
        private final SparkFlex motor2 = new SparkFlex(17, MotorType.kBrushless);
        private final RelativeEncoder encoder = motor1.getEncoder();
        private final RelativeEncoder encoder2 = motor2.getEncoder();
    
        public ShooterPowerSubsystem() {
           motor1.setInverted(true);
           motor2.setInverted(true);
        }
    
        public void runMotor(double speed) {
            motor1.set(speed);
            motor2.set(speed);
        }
    
        public void stopMotor() {
            motor1.set(0);
            motor2.set(0);
        }
    
        public double getVelocity() {
            return encoder.getVelocity(); 

        }

        public Command runMotorCommand() {
            return Commands.runOnce(
            () -> {
                runMotor(-5);


                
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
