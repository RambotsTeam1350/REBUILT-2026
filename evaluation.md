# FRC Team 1350 — 2026 Robot Competition Readiness Evaluation

**Date:** 2026-04-16  
**Branch:** main  
**Status: NOT COMPETITION READY**

---

## Critical Issues (Must Fix Before Deployment)

### 1. CAN ID 14 Latent Conflict (Not Currently Active)
**Files:**
- `src/main/java/frc/robot/subsystems/Intake/IntakeLevelSubsystem.java:26` — `new TalonFX(14)`
- `src/main/java/frc/robot/subsystems/Shooter/ShooterAimSubsystem.java:34` — `new TalonFX(14)`

Both files claim ID 14, but `ShooterAimSubsystem` is fully commented out in `RobotContainer.java` (lines 38, 75, 93) — the conflicting motor is never constructed at runtime. **This is not a blocking issue for competition.** However, if `ShooterAimSubsystem` is ever re-enabled, both motors would collide on the same ID. Reassign one before uncommenting.

---

### 2. Invalid CAN ID 999 in TestPIDMotorSubsystem (Not Currently Active)
**File:** `src/main/java/frc/robot/subsystems/TestPIDMotorSubsystem.java:21`

```java
private final TalonFX motor = new TalonFX(999);
```

The import and instantiation in `RobotContainer.java` (lines 40, 71) are both commented out, so this does not affect runtime. The class file remains a hazard if re-enabled. Not a blocking issue.

---

### 3. Shooter Distance-to-RPM Table
**File:** `src/main/java/frc/robot/subsystems/Shooter/ShooterPowerSubsystem.java:188–191`

Values are calibrated from physical testing. No action needed.

---

## High-Priority Issues

### 4. Missing Current Limits on Multiple Subsystems

The following subsystems do not configure explicit current limits on their TalonFX motors, leaving them at Phoenix6 defaults which may be too permissive:

| Subsystem | Motor ID(s) | Risk |
|-----------|-------------|------|
| TurretSubsystem | 18 | Mechanical hard stop could cause over-current |
| IntakeLevelSubsystem | 14 | Positional hard limits not paired with current protection |

All subsystems should explicitly set `StatorCurrentLimitConfigs` and `SupplyCurrentLimitConfigs`.

---

### 5. Intake Oscillation Command Nesting Issue
**File:** `src/main/java/frc/robot/RobotContainer.java:281–317`

`IntakeOcilateCommand()` returns a command with `.repeatedly()`. It is wrapped inside `Commands.startEnd()` on the left/right triggers. Scheduling a `repeatedly()` command via `startEnd` may not cancel cleanly when the trigger is released — the inner command's end behavior is not guaranteed to propagate. This needs a live test to confirm safe interrupt behavior.

---

### 6. Autonomous Default Path Unverified
**File:** `src/main/java/frc/robot/RobotContainer.java:182`

```java
AutoBuilder.buildAutoChooser("middle boring")
```

Confirm the path `middle boring` exists under `src/main/deploy/pathplanner/autos/`. If the file is missing, the auto chooser silently returns null and the robot sits still during autonomous.

---

## Medium-Priority Issues

### 7. Turret Angle Calculation is Unclear
**File:** `src/main/java/frc/robot/subsystems/TurretSubsystem.java:167–169`

```java
public double getTurretAngleBotRelative() {
    return 3 - getPoseEstimatorRotation();
}
```

The constant `3` has no documented meaning. This method is not currently called by any active command, but its existence is a hazard — if invoked accidentally, it will produce incorrect aiming angles. Either document and verify the math, or remove the method.

---

### 8. Unused Variable in CommandSwerveDrivetrain
**File:** `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java:73`

```java
Pose2d vision = LimelightHelpers.getBotPose2d_wpiBlue("limelight-fifteen");
```

This variable is assigned but never used. Minor, but could indicate incomplete vision-fusion logic.

---

## CAN ID Allocation Map

| ID(s) | Device | Status |
|-------|--------|--------|
| 0 | Pigeon2 IMU | ✓ |
| 1–4 | CANcoders (swerve) | ✓ |
| 5–12 | Drive/Steer motors (swerve) | ✓ |
| 13 | Indexer (ThroatAndIndexer) | ✓ |
| **14** | **IntakeLevelSubsystem AND ShooterAimSubsystem** | **CONFLICT** |
| 15–16 | (unused) | available |
| 17 | Climber Motor 1 (commented out) | — |
| 18 | Turret Motor | ✓ |
| 19 | Climber Motor 2 (commented out) | — |
| 20–33 | (unused) | available |
| 34 | Intake Wheel Motor | ✓ |
| 35 | CANdle (LEDs) | ✓ |
| 36–39 | (unused) | available |
| 40 | Shooter Motor 1 | ✓ |
| 41 | Shooter Motor 2 | ✓ |
| 42 | Backspin Motor | ✓ |
| 43–46 | (unused) | available |
| 47 | Throat Motor | ✓ |
| **999** | **TestPIDMotorSubsystem** | **INVALID** |

---

## Hardcoded Values to Verify on Hardware

| Value | Location | Notes |
|-------|----------|-------|
| Hub coordinates: Blue `(4.625594, 4.03479)`, Red `(11.915394, 4.03479)` | `TurretSubsystem.java:51–53` | Verify against 2026 field layout |
| Turret offset from robot center: `(-0.12, -0.12)` m | `TurretSubsystem.java:62–63` | Must match physical measurement |
| `turretPidControllerOffset = 0.38` | `TurretSubsystem.java:69` | Undocumented constant — verify meaning |
| Intake positions: down `−7.0 rot`, half `−3.1 rot`, up `0 rot` | `IntakeLevelSubsystem.java:84–88` | Must match mechanical hard stops |

---

## Pre-Competition Checklist

### Blocking — Fix Before Any Match
- [ ] ~~Calibrate shooter distance-to-RPM table~~ — done

### Strongly Recommended
- [ ] Reassign CAN ID in `ShooterAimSubsystem` before re-enabling it (currently conflicts with ID 14)
- [x] Add explicit `StatorCurrentLimitConfigs` to Turret and IntakeLevel motors — 40 A stator / 30 A supply, matching IntakeWheel and Throat subsystems
- [ ] Confirm "middle boring" auto path file exists and runs correctly
- [ ] Test intake oscillation command interrupt behavior on real hardware
- [ ] Verify all physical offset constants match the actual robot

### Before Each Event
- [ ] Full CAN bus health check (CTRE Tuner X)
- [ ] Calibrate both Limelights for field orientation
- [ ] End-to-end autonomous run on field elements
- [ ] Verify swerve PID tuning under match-condition carpet
