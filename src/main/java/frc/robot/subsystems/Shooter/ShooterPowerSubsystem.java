package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPowerSubsystem extends SubsystemBase {

    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFX backspinMotor;

    // Target speeds in RPM — adjust via tuning commands, setters, or the
    // distance-based calculator below once that is calibrated.
    public double shooterTargetRPM = 4500.0;
    public double backspinTargetRPM = 4500.0;

    // Idle speed kept on the flywheels between shots so the heavy flywheels
    // don't have to spin up from zero on the next shot. Tune this high enough
    // that spin-up time is acceptable, but low enough to avoid unnecessary
    // current draw and heat.
    private static final double STANDBY_RPM = 1500.0;

    // RPM step used by increase/decrease tuning commands
    private static final double RPM_STEP = 100.0;

    // How long a peak RPM error is held on the dashboard before resetting (seconds)
    private static final double PEAK_HOLD_SECONDS = 6.0;

    // Peak RPM error tracking — index 0 = motor1, 1 = motor2, 2 = backspin
    private final double[] peakErrors     = new double[3];
    private final double[] peakTimestamps = new double[3];

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
        double now = Timer.getFPGATimestamp();

        // Motor2 is commanded inverted, so its velocity signal is negative — use abs
        // for all three so errors are on the same scale (positive = under target).
        double actual1       = Math.abs(motor1.getVelocity().getValueAsDouble()       * 60.0);
        double actual2       = Math.abs(motor2.getVelocity().getValueAsDouble()       * 60.0);
        double actualBackspin = Math.abs(backspinMotor.getVelocity().getValueAsDouble() * 60.0);

        double[] errors = {
            shooterTargetRPM  - actual1,
            shooterTargetRPM  - actual2,
            backspinTargetRPM - actualBackspin
        };

        // Update peaks: promote if larger, or reset after the hold window expires.
        for (int i = 0; i < 3; i++) {
            if (Math.abs(errors[i]) > Math.abs(peakErrors[i])) {
                peakErrors[i]     = errors[i];
                peakTimestamps[i] = now;
            } else if (now - peakTimestamps[i] > PEAK_HOLD_SECONDS) {
                peakErrors[i]     = errors[i];
                peakTimestamps[i] = now;
            }
        }

        SmartDashboard.putNumber("Shooter Target RPM",    shooterTargetRPM);
        SmartDashboard.putNumber("Backspin Target RPM",   backspinTargetRPM);
        SmartDashboard.putNumber("Motor1 Actual RPM",     actual1);
        SmartDashboard.putNumber("Motor2 Actual RPM",     actual2);
        SmartDashboard.putNumber("Backspin Actual RPM",   actualBackspin);
        SmartDashboard.putNumber("Motor1 RPM Error",      errors[0]);
        SmartDashboard.putNumber("Motor2 RPM Error",      errors[1]);
        SmartDashboard.putNumber("Backspin RPM Error",    errors[2]);
        SmartDashboard.putNumber("Motor1 Peak RPM Error",   peakErrors[0]);
        SmartDashboard.putNumber("Motor2 Peak RPM Error",   peakErrors[1]);
        SmartDashboard.putNumber("Backspin Peak RPM Error", peakErrors[2]);
    }

    /**
     * Runs all three shooter motors at their current target RPMs using the
     * velocity PID. motor2 runs inverted due to opposite mechanical orientation.
     */
    public void runShooter() {
        motor1.setControl(velocityRequest.withVelocity(rpmToRps(shooterTargetRPM)));
        motor2.setControl(velocityRequest.withVelocity(-rpmToRps(shooterTargetRPM)));
        backspinMotor.setControl(velocityRequest.withVelocity(rpmToRps(backspinTargetRPM)));
    }

    /**
     * Runs all three shooter motors at speeds calculated from the given distance to
     * the hub. RPMs come from the interpolation table in updateRPMForDistance() and
     * are passed directly to the motor controllers — shooterTargetRPM and
     * backspinTargetRPM are not modified.
     *
     * @param distanceMeters straight-line distance from turret to hub
     *                       (use TurretSubsystem.getDistanceToHub())
     */
    public void runShooterWithAutoVelocity(double distanceMeters) {
        double[] rpms = updateRPMForDistance(distanceMeters);
        motor1.setControl(velocityRequest.withVelocity(rpmToRps(rpms[0])));
        motor2.setControl(velocityRequest.withVelocity(-rpmToRps(rpms[0])));
        backspinMotor.setControl(velocityRequest.withVelocity(rpmToRps(rpms[1])));
    }

    public void stopMotor() {
        motor1.set(0);
        motor2.set(0);
        backspinMotor.set(0);
    }

    /**
     * Holds the flywheels at a low idle speed between shots. Use this as the
     * "end" action on triggers so the heavy flywheels stay in motion and can
     * reach full speed faster on the next shot.
     * @return 
     */
    public void standbyMotor() {
        motor1.setControl(velocityRequest.withVelocity(rpmToRps(STANDBY_RPM)));
        motor2.setControl(velocityRequest.withVelocity(-rpmToRps(STANDBY_RPM)));
        backspinMotor.setControl(velocityRequest.withVelocity(rpmToRps(STANDBY_RPM)));
    }

    /**
     * Returns the desired shooter and backspin RPMs for a given distance to the
     * hub, interpolated from the measured shot data tables below.
     *
     * How to populate the tables:
     *   1. Place the robot at a known distance from the hub (measure in meters).
     *   2. Tune shooterTargetRPM manually until shots land consistently on target.
     *   3. Record the (distance, RPM) pair in the tables below.
     *   4. Repeat at several distances across the robot's expected shooting range.
     *
     * @param distanceMeters straight-line distance from turret to hub
     *                       (use TurretSubsystem.getDistanceToHub())
     * @return double[] { shooterRPM, backspinRPM }
     */
    public double[] updateRPMForDistance(double distanceMeters) {
        double[] distanceBreakpoints = { 1.6764, 2.1336, 2.4384, 2.8956, 3.3528, 3.6576, 3.9624, 4.1148, 4.2672, 4.4196, 4.572 };
        double[] shooterRPMPoints    = { 1300.00, 1400.00, 1500.00, 1600.00, 1700.00, 1800.00, 1900.00, 2000.00, 2100.00, 2200.00, 2400.00 };
        double[] backspinRPMPoints   = { 4500.00, 4500.00, 4500.00, 4500.00, 4500.00, 4500.00, 4500.00, 4500.00, 4500.00, 4500.00, 4500.00 };

        return new double[] {
            interpolate(distanceBreakpoints, shooterRPMPoints, distanceMeters),
            interpolate(distanceBreakpoints, backspinRPMPoints, distanceMeters)
        };
    }

    /**
     * Helper: linearly interpolates (or clamps) a value from a breakpoint
     * table. Used by updateRPMForDistance() once that method is calibrated.
     *
     * @param xs input breakpoints, must be sorted ascending
     * @param ys output values corresponding to each breakpoint
     * @param x  the input value to look up
     * @return interpolated (or clamped) output value
     */
    private static double interpolate(double[] xs, double[] ys, double x) {
        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
        for (int i = 0; i < xs.length - 1; i++) {
            if (x >= xs[i] && x <= xs[i + 1]) {
                double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return ys[ys.length - 1];
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
        return Commands.runOnce(this::runShooter);
    }

    public Command stopMotorCommand() {
        return Commands.runOnce(this::stopMotor);
    }
}
