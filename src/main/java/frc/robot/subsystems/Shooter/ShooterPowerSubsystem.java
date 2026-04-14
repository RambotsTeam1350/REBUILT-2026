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

    /**
     * Runs all three shooter motors at their current target RPMs using the
     * velocity PID. motor2 runs inverted due to opposite mechanical orientation.
     */
    public void runShooter() {
        motor1.setControl(velocityRequest.withVelocity(rpmToRps(shooterTargetRPM)));
        motor2.setControl(velocityRequest.withVelocity(-rpmToRps(shooterTargetRPM)));
        backspinMotor.setControl(velocityRequest.withVelocity(rpmToRps(backspinTargetRPM)));
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
     */
    public void standbyMotor() {
        motor1.setControl(velocityRequest.withVelocity(rpmToRps(STANDBY_RPM)));
        motor2.setControl(velocityRequest.withVelocity(-rpmToRps(STANDBY_RPM)));
        backspinMotor.setControl(velocityRequest.withVelocity(rpmToRps(STANDBY_RPM)));
    }

    /**
     * WIP: Updates shooterTargetRPM and backspinTargetRPM based on the robot's
     * current distance to the target (meters). Call this before runShooter() once
     * the lookup table is calibrated with real shot data.
     *
     * How to collect data:
     *   1. Place the robot at a known distance from the hub (measure in meters).
     *   2. Tune shooterTargetRPM manually until shots land consistently on target.
     *   3. Record the (distance, RPM) pair in the tables below.
     *   4. Repeat at several distances across the robot's expected shooting range.
     *   5. Uncomment the interpolation code and remove the flat fallback.
     *
     * @param distanceMeters straight-line distance from turret to target
     *                       (use TurretSubsystem.getDistanceToHub())
     */
    public void updateRPMForDistance(double distanceMeters) {
        // TODO: populate these tables with measured (distance → RPM) pairs.
        // Distances in meters; RPMs at those distances from real shot testing.
        // Add more rows as you collect data across the full shooting range.
        //
        // double[] distanceBreakpoints = { 2.0, 3.0, 4.0, 5.0, 6.0 };
        // double[] shooterRPMPoints    = { 3000, 3600, 4100, 4500, 4900 };
        // double[] backspinRPMPoints   = { 2500, 3000, 3400, 3800, 4200 };
        //
        // // Linear interpolation between the two nearest breakpoints.
        // // Clamps to the first/last value outside the measured range.
        // shooterTargetRPM = interpolate(distanceBreakpoints, shooterRPMPoints, distanceMeters);
        // backspinTargetRPM = interpolate(distanceBreakpoints, backspinRPMPoints, distanceMeters);

        // Flat fallback until the table above is filled in:
        // shooterTargetRPM and backspinTargetRPM are left at their current values.
    }

    /**
     * WIP helper: linearly interpolates (or clamps) a value from a breakpoint
     * table. Used by updateRPMForDistance() once that method is calibrated.
     *
     * @param xs input breakpoints, must be sorted ascending
     * @param ys output values corresponding to each breakpoint
     * @param x  the input value to look up
     * @return interpolated (or clamped) output value
     */
    // private static double interpolate(double[] xs, double[] ys, double x) {
    //     if (x <= xs[0]) return ys[0];
    //     if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
    //     for (int i = 0; i < xs.length - 1; i++) {
    //         if (x >= xs[i] && x <= xs[i + 1]) {
    //             double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
    //             return ys[i] + t * (ys[i + 1] - ys[i]);
    //         }
    //     }
    //     return ys[ys.length - 1];
    // }

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
