package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ThroatAndIndexerSubsystem extends SubsystemBase {

    private final TalonFX motorThroat;
    private final TalonFX motorIndexer;

    //public final StatusSignal<AngularVelocity> velocity;
    //public final StatusSignal<Angle> position;

    public double velocityDouble = 0.0;
    public static double positionDouble = 0.0;

    public ThroatAndIndexerSubsystem() {
        motorThroat = new TalonFX(47);
        motorIndexer = new TalonFX(33);

        //velocity = motor.getVelocity();
        //position = motor.getPosition();

     }

     public void runMotor(double speed) {
        motorThroat.set(speed);
        motorIndexer.set(speed);
    }

    public void stopMotor() {
        motorThroat.set(0);
        motorIndexer.set(0);
    }

    public void reverseMotor(double speed) {
        motorThroat.set(speed);
        motorIndexer.set(speed);
    }

      public Command runMotorCommand() {
        return Commands.runOnce(
            () -> {
                runMotor(.6);
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
            return Commands.sequence ( Commands.runOnce(
                () -> {
                    reverseMotor(-0.6);
                }
            ),
            Commands.waitSeconds(0.75),
            Commands.runOnce( () -> {
                    runMotor(0.6);
                }
            )
            );
        }
}

