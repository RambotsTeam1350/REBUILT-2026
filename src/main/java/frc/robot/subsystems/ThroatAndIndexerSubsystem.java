package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ThroatAndIndexerSubsystem  extends SubsystemBase {

    private final TalonFX motor;

    public final StatusSignal<AngularVelocity> velocity;
    public final StatusSignal<Angle> position;

    public double velocityDouble = 0.0;
    public static double positionDouble = 0.0;

    public ThroatAndIndexerSubsystem() {
        motor = new TalonFX(32);

        velocity = motor.getVelocity();
        position = motor.getPosition();

     }

     public void runMotor(double speed) {
        motor.set(speed);
    }

    public void stopMotor() {
        motor.set(0); 

    }



      public Command runMotorCommand() {
        return Commands.runOnce(
            () -> {
                runMotor(1);
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
