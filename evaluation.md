# FRC Team 1350 — 2026 Robot Code Evaluation

**Evaluated by:** Claude Code (claude-sonnet-4-6)
**Date:** 2026-03-16
**Branch:** main

---

## Summary

The codebase has a solid architectural foundation (command-based, Phoenix 6, PathPlanner, MegaTag2 vision) but contains several **critical bugs that will prevent the robot from functioning correctly at competition**. The most severe are duplicate hardware instantiations and a CAN ID conflict that will cause erratic behavior or an immediate brownout. These must be fixed before any practice matches.

---

## Critical Issues (Fix Before Powering On)

### 1. Duplicate Drivetrain Instantiation
**Files:** [Robot.java:26](src/main/java/frc/robot/Robot.java#L26), [RobotContainer.java:65](src/main/java/frc/robot/RobotContainer.java#L65)

`TunerConstants.createDrivetrain()` is called **twice** — once in `Robot` and once in `RobotContainer`. This instantiates all 8 swerve TalonFX motors, 4 CANcoders, and the Pigeon2 twice. The two drivetrains have completely independent pose estimators. The turret in `Robot.java` uses `Robot.drivetrain.getPoseEstimator()` while all joystick bindings use `RobotContainer.drivetrain`. These will never agree on robot position. This also doubles CAN bus traffic during initialization and risks device configuration conflicts.

**Fix:** Remove `Robot.drivetrain` entirely. Pass `RobotContainer`'s drivetrain (or its pose estimator) to any subsystem that needs it. The canonical pattern is for `RobotContainer` to own all subsystems and the single drivetrain.

---

### 2. Duplicate TurretSubsystem Instantiation
**Files:** [Robot.java:33](src/main/java/frc/robot/Robot.java#L33), [RobotContainer.java:79](src/main/java/frc/robot/RobotContainer.java#L79)

`TurretSubsystem` is instantiated in **both** `Robot` and `RobotContainer`, both controlling TalonFX ID 13. The instance in `Robot.java` is not even registered with the CommandScheduler (line 34 is commented out), so its `periodic()` runs but its commands cannot be properly scheduled. The `RobotContainer` instance is the one attached to joystick bindings. Two subsystems fighting over one motor will produce undefined behavior.

**Fix:** Remove `turretSubsystem` from `Robot.java`. Only `RobotContainer` should own subsystems.

---

### 3. Duplicate IntakeLevelSubsystem Instantiation
**Files:** [Robot.java:27](src/main/java/frc/robot/Robot.java#L27), [RobotContainer.java:57](src/main/java/frc/robot/RobotContainer.java#L57)

`IntakeLevelSubsystem` is instantiated twice (both use TalonFX ID 17). Both will register as separate `SubsystemBase` instances, the CommandScheduler will run `periodic()` twice per loop, and commands from each will conflict with each other.

**Fix:** Remove `intakeLevelSubsystem` from `Robot.java`.

---

### 4. CAN ID Conflict — Climber vs. Back-Left Drive Motor
**Files:** [ClimberSubsystem.java:18](src/main/java/frc/robot/subsystems/ClimberSubsystem.java#L18), [TunerConstantsThorBot.java:151](src/main/java/frc/robot/generated/TunerConstantsThorBot.java#L151)

`ClimberSubsystem` uses `new TalonFX(15)`. The back-left swerve drive motor (`kBackLeftDriveMotorId`) is also ID 15. **Two TalonFX devices on the same CAN bus with the same ID.** This will cause one or both motors to fail to configure, random control conflicts, and likely a brownout when both try to respond to the same CAN frames. The comment in `ClimberSubsystem.java` reads _"change the moter ID for the climber when we know what it is"_ — this was never resolved.

**Fix:** Assign the climber motor a unique CAN ID (e.g., 20) and update the physical motor's ID using Phoenix Tuner X.

---

### 5. Duplicate Pigeon2 Instantiation
**Files:** [Robot.java:23](src/main/java/frc/robot/Robot.java#L23), [CommandSwerveDrivetrain.java:62](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java#L62)

`new Pigeon2(0, "rio")` appears in both files. Two Java objects for the same physical IMU. `Robot.robotInit()` calls `pigeon.setYaw(0)` — but this affects Robot's own Pigeon instance, not the one used by the drivetrain, so the reset has no effect on the drivetrain's heading. Additionally, two Phoenix 6 objects for the same device causes duplicated signal subscriptions and wasted CAN bandwidth.

**Fix:** Remove `pigeon` from `Robot.java` entirely. Use `drivetrain.getPigeon2()` (available from the CTRE base class) if you need gyro access outside the drivetrain.

---

## High-Priority Bugs

### 6. `TurretSubsystem.periodic()` Discards Command Object — Turret Never Continuously Aims
**File:** [TurretSubsystem.java:110](src/main/java/frc/robot/subsystems/TurretSubsystem.java#L110)

```java
TurretAutoAimToHub(); // returns a Command — result is silently discarded
```

`TurretAutoAimToHub()` returns a `Command`. Calling it without `.schedule()` creates and immediately discards a Command object every 20ms. The motor never moves from this call. If the intent is continuous auto-aiming in `periodic()`, use the imperative version:

```java
turretAutoAimToHubImmediate(); // directly commands the motor
```

---

### 7. `AlignToHub` Misinterprets `txnc` — Wrong Angle Scale
**File:** [AlignToHub.java:102-111](src/main/java/frc/robot/commands/AlignToHub.java#L102)

```java
double horizontalAngleOffset = hubTag.txnc;
double angleToTagFromCamera = horizontalAngleOffset * (cameraHorizontalFOV / 2.0); // ← wrong
```

`RawFiducial.txnc` is already the horizontal angle in **degrees** (not normalized to ±1). The code treats it as normalized `[-1, 1]` and multiplies by `FOV/2 = 31.65°`, producing an angle up to `~31°` when it should be at most `~31.65°` — but worse, at small angles (e.g., 5°) it outputs `5 × 31.65 = 158°`. The turret will command the wrong direction.

**Fix:** Use `hubTag.txnc` directly as the angle offset in degrees. No multiplication needed:

```java
double angleToTagFromCamera = hubTag.txnc; // already in degrees
```

---

### 8. `ShooterAimSubsystem.degreesToEncoderUnits()` Uses Phoenix 5 Scale (2048) in Phoenix 6
**File:** [ShooterAimSubsystem.java:84-88](src/main/java/frc/robot/subsystems/Shooter/ShooterAimSubsystem.java#L84)

```java
double unitsPerRevolution = 2048; // ← Phoenix 5 constant, not applicable to Phoenix 6
return (degrees / 360.0) * unitsPerRevolution * gearBoxRatio;
```

Phoenix 6 TalonFX position control uses **rotations**, not raw encoder counts. The factor of 2048 will send positions ~2048× too large, slamming the shooter angle mechanism into its hard stop immediately.

**Fix:**
```java
return (degrees / 360.0) * gearBoxRatio;
```

The same dead variable (`unitsPerRevolution = 2048`) also appears in `TurretSubsystem.degreesToEncoderUnits()` at [TurretSubsystem.java:164](src/main/java/frc/robot/subsystems/TurretSubsystem.java#L164) — it is declared but not used in the computation there, so the turret conversion is actually correct by accident.

---

### 9. `getAngleToTarget()` Mixes Degrees and Radians
**File:** [TurretSubsystem.java:142-144](src/main/java/frc/robot/subsystems/TurretSubsystem.java#L142)

```java
public double getAngleToTarget() {
    return (getPoseEstimatorRotation() - getTargetRotation()); // degrees - radians
}
```

`getPoseEstimatorRotation()` returns **degrees**; `getTargetRotation()` returns **radians** (raw output of `Math.atan2`). The result is meaningless. This method is called in `periodic()` but the return value is unused, so it has no runtime impact — but if it's ever used for control it will produce garbage.

**Fix:** Either convert `getTargetRotation()` to degrees with `Math.toDegrees()`, or use the existing `GetTurretToHub` utility for angle calculation.

---

## Medium-Priority Issues

### 10. Unconditional `setVisionMeasurementStdDevs(0.00001, ...)` in `periodic()`
**File:** [CommandSwerveDrivetrain.java:386](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java#L386)

```java
poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.00001, 0.00001, 9999));
```

This is called **before** the vision measurement check. If no AprilTags are visible (and `addVisionMeasurement` is skipped), the estimator is left with X/Y standard deviations of 0.00001 m — effectively treating odometry as perfect. This will resist all future vision corrections until the next update cycle where tags are seen. The line should be removed; the per-measurement `setVisionMeasurementStdDevs` calls on lines 403 and 410 are correct and sufficient.

---

### 11. Gear Ratio Comment Mismatch — Turret
**File:** [TurretSubsystem.java:47](src/main/java/frc/robot/subsystems/TurretSubsystem.java#L47)

```java
double gearBoxRatio = 3.0; // Assuming a 9:1 gear ratio for the turret
```

The comment says 9:1 but the value is 3.0. Additionally, the conversion formula includes an empirical correction factor `(8.5810546875/9)`, suggesting the gear ratio was measured to be approximately 2.86 rotations per revolution instead of 3.0. The correct value should be measured physically with Phoenix Tuner X and set directly — not patched with a magic constant.

---

### 12. Unfinished Geometry Methods in TurretSubsystem
**File:** [TurretSubsystem.java:147-158](src/main/java/frc/robot/subsystems/TurretSubsystem.java#L147)

Three methods are broken/unfinished:

- **`getPositionTurretonField()`** always returns `new Translation2d()` (0, 0). Never implemented.
- **`getTurretAngleBotRelative()`** returns `3 - getPoseEstimatorRotation()` — subtracting degrees from what appears to be a radian constant. Meaningless.
- **`getDistanceBotonTurretFieldRelative()`** applies an incomplete rotation matrix (missing cross terms). The correct transform is `x' = x·cos(θ) - y·sin(θ)`, `y' = x·sin(θ) + y·cos(θ)`.

None of these are currently used in the active control path, but they should be fixed or removed before being hooked in.

---

### 13. `System.out.println` in Periodic Loops
**Files:** [CommandSwerveDrivetrain.java:413](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java#L413), [ShooterAimSubsystem.java:62](src/main/java/frc/robot/subsystems/Shooter/ShooterAimSubsystem.java#L62), [AlignToHub.java:153](src/main/java/frc/robot/commands/AlignToHub.java#L153)

Three separate `System.out.println` calls in code that executes every loop cycle (50 Hz). On the roboRIO, stdout is buffered but still consumes CPU time and can cause loop overruns under load. Replace with `SmartDashboard.putString/putNumber` or WPILib's `DataLog` for diagnostics. Remove or comment out before competition.

---

### 14. `ShooterAimSubsystem` Is Never Instantiated
**File:** [ShooterAimSubsystem.java](src/main/java/frc/robot/subsystems/Shooter/ShooterAimSubsystem.java)

The shooter angle subsystem exists with a full `AngleAdjust()` command and `getAngleAdjusterTheta()` calculation but is never instantiated in `RobotContainer` or `Robot`. Without it, the shooter angle is never controlled.

---

### 15. No Software Limits on Turret TalonFX
**File:** [TurretSubsystem.java:75-91](src/main/java/frc/robot/subsystems/TurretSubsystem.java#L75)

The turret limits (±90°) are enforced only in `clampTurretAngle()` before commanding MotionMagic. If any code path sends an unclamped value to the motor (e.g., `TurretToMaxPosition()` at line 196 sends 360°), the turret can over-rotate and damage the wiring harness or mechanism. Configure `SoftwareLimitSwitchConfigs` in the TalonFX configuration as a hardware backstop:

```java
cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToEncoderUnits(90);
cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToEncoderUnits(-90);
cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
```

---

## Low-Priority / Code Quality

### 16. Duplicate Import in RobotContainer
**File:** [RobotContainer.java:28,38](src/main/java/frc/robot/RobotContainer.java#L28)

`import frc.robot.commands.AlignToHub;` appears on both line 28 and line 38. Will generate a compiler warning; clean up.

### 17. Unused Imports and Fields in RobotContainer
**File:** [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java)

- `import edu.wpi.first.wpilibj.Joystick;` — unused
- `import edu.wpi.first.math.estimator.PoseEstimator;` — unused
- `import com.revrobotics.spark.SparkFlex;` — unused (motor commented out)
- `private LimelightTarget_Detector limelight` (line 68) — instantiated but never referenced

### 18. `TestPIDMotorSubsystem` Uses CAN Resources and Does Nothing
**File:** [RobotContainer.java:63](src/main/java/frc/robot/RobotContainer.java#L63)

`pidcontroler` is instantiated and registered with the scheduler but all commands are commented out. It occupies TalonFX ID 33 and runs `periodic()` every loop for no reason. Remove or leave only if actively needed for testing.

### 19. Dead Code in `CommandSwerveDrivetrain`
**File:** [CommandSwerveDrivetrain.java:450-480](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java#L450)

The `drive()` method is never called anywhere in the codebase. Inside it, `adjustedTranslation` is computed but `translation` (not `adjustedTranslation`) is passed to `ChassisSpeeds.fromFieldRelativeSpeeds()`. The module state application block is commented out. Also `initialLeftDistance`, `initialRightDistance`, and the class-level `vision` field are all declared but never used.

### 20. Alliance-Specific Hub Tag IDs May Not Match 2026 Game
**File:** [AlignToHub.java:25-26](src/main/java/frc/robot/commands/AlignToHub.java#L25)

```java
private static final int[] RED_HUB_TAGS = {9, 10};
private static final int[] BLUE_HUB_TAGS = {24, 25};
```

Verify these tag IDs against the [2026 Game Manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf). AprilTag layouts change each season and 24/25 may not correspond to hub targets in the 2026 game.

### 21. No Pose Reset at Autonomous Start
**File:** [Robot.java:61-67](src/main/java/frc/robot/Robot.java#L61)

`autonomousInit()` schedules the auto command but never seeds the pose estimator with the path's starting pose. PathPlanner will call `resetPose()` if the auto has a starting pose configured, but if paths don't define one, the robot will execute paths from whatever the current estimated position is. Ensure all competition autos define a starting pose, or explicitly reset in `autonomousInit()`.

### 22. Only Example/Placeholder Paths in PathPlanner
**File:** [src/main/deploy/pathplanner/autos/](src/main/deploy/pathplanner/autos/)

The only deployed auto is "Example Auto" using placeholder paths. No competition-ready autonomous routines exist.

---

## CAN ID Summary

| ID | Device | Conflict? |
|----|--------|-----------|
| 0  | Pigeon2 (×2 instantiated) | Yes — two objects, one device |
| 1  | Front Right CANcoder | OK |
| 2  | Front Left CANcoder | OK |
| 3  | Back Left CANcoder | OK |
| 4  | Back Right CANcoder | OK |
| 5  | Front Left Drive TalonFX | OK |
| 6  | Front Right Drive TalonFX | OK |
| 7  | Front Left Steer TalonFX | OK |
| 8  | Back Right Drive TalonFX | OK |
| 9  | Front Right Steer TalonFX | OK |
| 10 | Back Right Steer TalonFX | OK |
| 11 | Back Left Steer TalonFX | OK |
| 13 | Turret TalonFX (×2 instantiated) | Yes — two subsystems |
| 14 | Shooter Aim TalonFX | OK (never instantiated) |
| **15** | **Back Left Drive TalonFX AND Climber TalonFX** | **CONFLICT** |
| 16 | Shooter SparkFlex #1 | OK |
| 17 | IntakeLevelSubsystem TalonFX (×2 instantiated) + Shooter SparkFlex #2 | TalonFX duplicated; SparkFlex and TalonFX share ID 17 but are different device types (OK) |
| 18 | IntakeWheel TalonFX | OK |
| 32 | Throat/Indexer TalonFX | OK |
| 33 | TestPIDMotor TalonFX | OK |

---

## Recommended Fix Priority

1. Fix CAN ID 15 conflict (climber vs. back-left drive) — assign climber a new ID in Tuner X
2. Remove `Robot.java`'s `drivetrain`, `turretSubsystem`, `intakeLevelSubsystem`, and `pigeon` fields — `RobotContainer` should own all subsystems
3. Fix `TurretSubsystem.periodic()` to call `turretAutoAimToHubImmediate()` instead of `TurretAutoAimToHub()`
4. Fix `AlignToHub` to use `txnc` directly (already in degrees)
5. Fix `ShooterAimSubsystem.degreesToEncoderUnits()` to remove the ×2048 multiplier
6. Remove all `System.out.println` from periodic/execute methods
7. Instantiate `ShooterAimSubsystem` in `RobotContainer` and bind it to a button
8. Add TalonFX software limits to TurretSubsystem
9. Fix `getAngleToTarget()` unit mismatch (or remove if unused)
10. Build and deploy competition autonomous routines
