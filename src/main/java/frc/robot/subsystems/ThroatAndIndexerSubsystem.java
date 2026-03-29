package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
        motorIndexer = new TalonFX(13);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLimitEnable(true);
        cfg.OpenLoopRamps = new OpenLoopRampsConfigs()
            .withDutyCycleOpenLoopRampPeriod(0.3); // seconds from 0 to full output
        motorThroat.getConfigurator().apply(cfg);
        motorIndexer.getConfigurator().apply(cfg);
     }

     public void runMotor() {
        motorThroat.set(0.6);
        motorIndexer.set(0.4); //changed at UNH
    }

    public void stopMotorThroat() {
        motorThroat.set(0);
    }

    public void stopMotorIndexer() {
        motorIndexer.set(0);
    }

    public void reverseMotor(double speed) {
        motorThroat.set(speed);
        motorIndexer.set(speed);
    }

      public Command runMotorCommand() {
        return Commands.sequence(
        Commands.runOnce(
            () -> {
                runMotor();
            })
        );
    }

     public Command stopMotorCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                stopMotorIndexer();
            }),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> {
                stopMotorThroat();
            })

        );
     }

        public Command reverseMotorCommand() {
            return Commands.sequence ( Commands.runOnce(
                () -> {
                    reverseMotor(-0.4);
                }
            ),
            Commands.waitSeconds(0.2),
            Commands.runOnce( () -> {
                    runMotor();
                }
            )
            );
        }
}

