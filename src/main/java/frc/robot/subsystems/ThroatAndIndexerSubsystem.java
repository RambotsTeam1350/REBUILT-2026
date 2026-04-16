package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ThroatAndIndexerSubsystem extends SubsystemBase {

    private final TalonFX motorThroat;
    private final TalonFX motorIndexer;

    // public final StatusSignal<AngularVelocity> velocity;
    // public final StatusSignal<Angle> position;

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

    // Interval between reverses (seconds) and how long to reverse for (seconds)
    private static final double DEFAULT_REVERSE_INTERVAL = 3.0;
    private static final double DEFAULT_REVERSE_DURATION = 0.2;

    // Timer and state used by the periodic-reverse command implementation
    private final Timer periodicReverseTimer = new Timer();
    private boolean periodicReversing = false;

    public void runMotor() {
        motorThroat.set(0.6);
        motorIndexer.set(0.4); // changed at UNH
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
                        }));
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
        return Commands.sequence(Commands.runOnce(
                () -> {
                    reverseMotor(-0.4);
                }),
                Commands.waitSeconds(0.2),
                Commands.runOnce(() -> {
                    runMotor();
                }));
    }

    /**
     * Command that runs the motors forward continuously and, every {@code interval}
     * seconds, briefly reverses them for {@code reverseDuration} seconds.
     * This is implemented as a single Command so it is the only command that
     * requires this subsystem (avoids parallel requirement conflicts).
     */
    public Command runMotorWithPeriodicReverseCommand() {
        final double interval = DEFAULT_REVERSE_INTERVAL;
        final double reverseDuration = DEFAULT_REVERSE_DURATION;

        // Start action: reset timer and start forward motors
        Runnable start = () -> {
            periodicReverseTimer.reset();
            periodicReverseTimer.start();
            periodicReversing = false;
            runMotor();
        };

        // End action: stop motors and timer
        Runnable end = () -> {
            stopMotorThroat();
            stopMotorIndexer();
            periodicReverseTimer.stop();
        };

        // Repeating action: called each scheduler cycle while this command is active
        Runnable repeating = () -> {
            double t = periodicReverseTimer.get();
            if (!periodicReversing) {
                if (t >= interval) {
                    periodicReversing = true;
                    periodicReverseTimer.reset();
                    reverseMotor(-0.4);
                }
            } else {
                if (t >= reverseDuration) {
                    periodicReversing = false;
                    periodicReverseTimer.reset();
                    runMotor();
                }
            }
        };

        // Use a single parallel command: a start/end wrapper and a repeating runnable.
        // The command requires this subsystem so no other command can conflict.
        return Commands.parallel(
                Commands.startEnd(start, end, this),
                Commands.run(repeating, this)
        );
    }
}
