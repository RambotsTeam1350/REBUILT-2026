package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPowerSubsystem extends SubsystemBase {

    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFX backspinMotor;

    // Target speeds in RPM — adjust via tuning commands or setters
    public double shooterTargetRPM = 4500.0;
    public double backspinTargetRPM = 4500.0;

    // RPM step used by increase/decrease tuning commands
    private static final double RPM_STEP = 100.0;

    // Reusable velocity request — slot 0 is configured below
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private final Notifier speedNotifier;

    public ShooterPowerSubsystem() {
        motor1 = new TalonFX(40);
        motor2 = new TalonFX(41);
        backspinMotor = new TalonFX(42);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Closed-loop velocity gains.
        // kV  — feedforward: volts per RPS to maintain steady speed. Start here and
        //        tune until free-spin speed matches target with near-zero kP error.
        // kS  — static friction offset: small constant to overcome stiction.
        // kA  — acceleration feedforward: extra volts per RPS/s of acceleration.
        //        This is the key gain for flywheel inertia — increase it if recovery
        //        after a shot is still too slow.
        // kP  — proportional: drives remaining steady-state error. Increase if the
        //        wheel consistently runs below target; back off if it oscillates.
        cfg.Slot0 = new Slot0Configs()
                .withKS(0.25)   // V — tune first on a still motor
                .withKV(0.12)   // V/RPS — tune to match actual free speed at 12 V
                .withKA(0.02)   // V/(RPS/s) — increase to fight flywheel inertia
                .withKP(0.5)    // V/RPS error — increase for faster recovery
                .withKI(0)
                .withKD(0);

        cfg.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true);

        motor1.getConfigurator().apply(cfg);
        motor2.getConfigurator().apply(cfg);
        backspinMotor.getConfigurator().apply(cfg);

        speedNotifier = new Notifier(this::updateDashboard);
        speedNotifier.startPeriodic(0.5);
    }

    private static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("Shooter Target RPM", shooterTargetRPM);
        SmartDashboard.putNumber("Backspin Target RPM", backspinTargetRPM);
        SmartDashboard.putNumber("Motor1 Actual RPM", motor1.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Motor2 Actual RPM", motor2.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Backspin Actual RPM", backspinMotor.getVelocity().getValueAsDouble() * 60.0);
    }

    // motor2 runs inverted (opposite mechanical orientation to motor1)
    public void runMotor1(double speed) {
        motor1.setControl(velocityRequest.withVelocity(rpmToRps(shooterTargetRPM)));
    }

    public void runMotor2(double speed) {
        motor2.setControl(velocityRequest.withVelocity(-rpmToRps(shooterTargetRPM)));
    }

    public void runBackspinMotor(double speed) {
        backspinMotor.setControl(velocityRequest.withVelocity(rpmToRps(backspinTargetRPM)));
    }

    public void stopMotor() {
        motor1.set(0);
        motor2.set(0);
        backspinMotor.set(0);
    }

    public void setShooterRPM(double rpm) {
        shooterTargetRPM = rpm;
    }

    public void setBackspinRPM(double rpm) {
        backspinTargetRPM = rpm;
    }

    public Command increaseLowerWheelSpeed() {
        return Commands.runOnce(() -> shooterTargetRPM += RPM_STEP);
    }

    public Command decreaseLowerWheelSpeed() {
        return Commands.runOnce(() -> shooterTargetRPM -= RPM_STEP);
    }

    public Command increaseBackspinWheelSpeed() {
        return Commands.runOnce(() -> backspinTargetRPM += RPM_STEP);
    }

    public Command decreaseBackspinWheelSpeed() {
        return Commands.runOnce(() -> backspinTargetRPM -= RPM_STEP);
    }

    public Command runMotorCommand() {
        return Commands.runOnce(() -> {
            runMotor1(0);
            runMotor2(0);
            runBackspinMotor(0);
        });
    }

    public Command stopMotorCommand() {
        return Commands.runOnce(this::stopMotor);
    }
}
