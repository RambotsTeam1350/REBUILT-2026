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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPowerSubsystem extends SubsystemBase {

    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFX backspinMotor;
    public double lowerWheelSpeed;
    public double backspinWheelSpeed;
    private final Notifier speedNotifier;

    public ShooterPowerSubsystem() {
        motor1 = new TalonFX(40);
        motor2 = new TalonFX(41);
        backspinMotor = new TalonFX(42);
        lowerWheelSpeed = 0.8;
        backspinWheelSpeed = -0.8;

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true);
        motor1.getConfigurator().apply(cfg);
        motor2.getConfigurator().apply(cfg);
        backspinMotor.getConfigurator().apply(cfg);

        speedNotifier = new Notifier(this::updateMotorSpeed);
        // update twice per second
        speedNotifier.startPeriodic(0.5);

    }

    private void updateMotorSpeed() {
        SmartDashboard.putNumber("Lower Wheel Speed", lowerWheelSpeed);
        SmartDashboard.putNumber("Backspin Wheel Speed", backspinWheelSpeed);
    }

    public void runMotor1(double speed) {
        motor1.set(lowerWheelSpeed);
    }

    public void runMotor2(double speed) {
        motor2.set(-lowerWheelSpeed);
    }

    public void runBackspinMotor(double speed) {
        backspinMotor.set(-backspinWheelSpeed);
    }

    public void stopMotor() {
        motor1.set(0);
        motor2.set(0);
        backspinMotor.set(0);
    }

    /**
     * Default assumed max RPM for the shooter motor (Falcon 500 free speed).
     * If your motor or gearing differs, pass a different maxRPM into rpmToPercent.
     */
    private static final double DEFAULT_SHOOTER_MAX_RPM = 6000.0;

    /**
     * Convert an RPM target into a normalized percent output for
     * {@code motor.set(percent)}.
     * Linear mapping: percent = rpm / maxRPM, clamped to [-1, 1].
     * This is open-loop — for accurate speed control use a closed-loop velocity
     * API.
     *
     * @param rpm    desired rotations per minute (positive forward)
     * @param maxRPM the RPM that corresponds to 100% output (motor free speed *
     *               gear ratio)
     * @return normalized percent in range [-1.0, 1.0]
     */
    public double rpmToPercent(double rpm, double maxRPM) {
        if (maxRPM == 0) {
            return 0.0;
        }
        double pct = rpm / maxRPM;
        if (pct > 1.0)
            return 1.0;
        if (pct < -1.0)
            return -1.0;
        return pct;
    }

    /**
     * Convenience overload using a reasonable default motor free-speed.
     * 
     * @param rpm target RPM
     * @return percent output [-1,1]
     */
    public double rpmToPercent(double rpm) {
        return rpmToPercent(rpm, DEFAULT_SHOOTER_MAX_RPM);
    }

    /**
     * Command both shooter motors to an approximate open-loop RPM target.
     * motor2 is inverted relative to motor1 on this robot (see existing
     * runMotorCommand).
     * 
     * @param rpm target RPM (positive forward)
     */
    public void setShooterRPM(double rpm) {
        double pct = rpmToPercent(rpm);
        motor1.set(pct);
        motor2.set(-pct);
    }

    /**
     * Set the backspin motor approximate RPM using the same conversion.
     * 
     * @param rpm target RPM for backspin motor
     */
    public void setBackspinRPM(double rpm) {
        backspinMotor.set(rpmToPercent(rpm));
    }

    public Command increaseLowerWheelSpeed() {
        return Commands.runOnce(() -> {
            lowerWheelSpeed += 0.05;
        });
    }

    public Command decreaseLowerWheelSpeed() {
        return Commands.runOnce(() -> {
            lowerWheelSpeed -= 0.05;
        });
    }

    public void setLowerWheelSpeed(double speed) {
        lowerWheelSpeed = speed;
        if (lowerWheelSpeed > 1.0) {
            lowerWheelSpeed = 1.0;
        }
        if (lowerWheelSpeed < 0.1) {
            lowerWheelSpeed = 0.1;
        }
    }

    public Command increaseBackspinWheelSpeed() {
        return Commands.runOnce(() -> {
            backspinWheelSpeed -= 0.05;
        });
    }

    public Command decreaseBackspinWheelSpeed() {
        return Commands.runOnce(() -> {
            backspinWheelSpeed += 0.05;
        });
    }

    public void setBackspinWheelSpeed(double speed) {
        backspinWheelSpeed = speed;
        if (backspinWheelSpeed < -1.0) {
            backspinWheelSpeed = -1.0;
        }
        if (backspinWheelSpeed > -0.1) {
            backspinWheelSpeed = -0.1;
        }
    }

    public Command runMotorCommand() {
        return Commands.runOnce(
                () -> {
                    runMotor1(1);
                    runMotor2(-1);
                    runBackspinMotor(-1);
                });
    }

    public Command stopMotorCommand() {
        return Commands.runOnce(
                () -> {
                    stopMotor();
                });
    }
}
