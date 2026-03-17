# CANcoder: Persistent Turret Position Across Power Cycles

The current setup relies entirely on the TalonFX's internal encoder, which resets to 0 on every power cycle. Adding a CANcoder lets the turret always know its absolute position — no re-zeroing before matches.

## How it works

A CANcoder uses a hall-effect sensor reading a magnet — it always knows the absolute shaft angle, even with power off. Configuring the TalonFX to use it as its feedback source (`FusedCANcoder` with Pro, or `RemoteCANcoder` without) seeds the motor's position from the absolute reading at boot.

## One-time calibration (do this first)

1. Physically rotate the turret to your desired **zero position** (e.g., pointing straight back)
2. Open **Phoenix Tuner X → CANcoder → Signals tab** and note the `Absolute Position` value in rotations (e.g., `0.342`)
3. Set `MagnetOffset = -0.342` in the config below so that zero = home

## Code changes to `TurretSubsystem.java`

**Add imports:**

```java
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
```

**Add the CANcoder field alongside the motor:**

```java
private final TalonFX motor;
private final CANcoder turretEncoder;  // ADD THIS

// TODO: Set this to -rawAbsolutePosition after calibration in Tuner X
private static final double CANCODER_MAGNET_OFFSET_ROTATIONS = 0.0;
private static final int CANCODER_CAN_ID = 19; // pick an unused CAN ID
```

**Replace the constructor's config block:**

```java
public TurretSubsystem(SwerveDrivePoseEstimator poseEstimator) {
    this.poseEstimator = poseEstimator;
    motor = new TalonFX(13);
    turretEncoder = new CANcoder(CANCODER_CAN_ID);

    // --- CANcoder configuration ---
    CANcoderConfiguration encoderCfg = new CANcoderConfiguration();
    MagnetSensorConfigs magnet = encoderCfg.MagnetSensor;

    // AbsoluteSensorDiscontinuityPoint = 0.5 means range is [-0.5, 0.5) rotations.
    // This matches the turret's [-90°, 90°] physical range (±0.25 rot) with room to spare.
    magnet.AbsoluteSensorDiscontinuityPoint = 0.5;

    // Flip this to Clockwise_Positive if the encoder reads backward vs. motor direction
    magnet.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    // This is what eliminates re-zeroing: offset bakes in the home position permanently.
    magnet.MagnetOffset = CANCODER_MAGNET_OFFSET_ROTATIONS;

    turretEncoder.getConfigurator().apply(encoderCfg);

    // --- TalonFX configuration ---
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.Slot0.kP = 4.8;
    cfg.Slot0.kI = 0;
    cfg.Slot0.kD = 0.1;
    cfg.Slot0.kV = 0.12;
    cfg.Slot0.kA = 0.01;
    cfg.Slot0.kS = 0.25;

    // Tell the TalonFX to use the CANcoder as its position source.
    // FusedCANcoder (Phoenix Pro) is preferred — it blends absolute position with
    // motor velocity for accuracy. Use RemoteCANcoder if you don't have Pro.
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // or RemoteCANcoder
    cfg.Feedback.FeedbackRemoteSensorID = CANCODER_CAN_ID;

    // If CANcoder is on the turret OUTPUT shaft (1:1 with the turret):
    cfg.Feedback.SensorToMechanismRatio = 1.0;   // CANcoder rotations per turret rotation
    cfg.Feedback.RotorToSensorRatio = gearBoxRatio; // motor rotations per CANcoder rotation

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 7;
    mm.MotionMagicAcceleration = 80;
    mm.MotionMagicJerk = 1600;
    motor.getConfigurator().apply(cfg);

    this.motorPosition = this.motor.getPosition();
}
```

**Update `degreesToEncoderUnits` and `encoderUnitsToDegrees`:**

With a CANcoder as feedback, the TalonFX reports position in **rotations of the mechanism** (the turret), not raw rotor counts. Simplify the conversion:

```java
private double degreesToEncoderUnits(double degrees) {
    // TalonFX now reports in turret rotations (via CANcoder feedback)
    return degrees / 360.0;
}

private double encoderUnitsToDegrees(double rotations) {
    return rotations * 360.0;
}
```

## Summary of what changes and why

| Before | After |
|---|---|
| TalonFX internal encoder, resets to 0 at boot | CANcoder absolute position seeds TalonFX on every power-up |
| Complex empirical correction (`8.5810546875/9`) | Clean 1:1 CANcoder-to-turret ratio |
| Must manually re-zero before every match | Magnet offset baked in — plug in and go |

## Where to mount the CANcoder

Put it on the **turret output shaft** (after the gearbox). If you put it on the motor shaft (before gearbox), swap the ratios:

```java
cfg.Feedback.SensorToMechanismRatio = gearBoxRatio;
cfg.Feedback.RotorToSensorRatio = 1.0;
```

Output-shaft mounting is preferred because it directly measures turret angle without depending on gearbox accuracy.
