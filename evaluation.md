# Competition Code Evaluation — FRC Team 1350 La Salle Robotics 2026

**Evaluated:** 2026-04-15  
**Branch:** aiming-fixes

Issues are grouped by severity. Fix blockers before any match. High severity items should be addressed before eliminations.

---

## ~~BLOCKER~~ — All blockers resolved (commits fcb7b1d, b230672, 2c84ea3)

### ~~1. `IntakeOcilateCommand()` is silently discarded — intake never oscillates~~ ✓ FIXED

**Fixed in:** `fcb7b1d`

`IntakeOcilateCommand()` was called inside a void lambda inside `Commands.startEnd()`, causing the returned Command to be immediately discarded and never scheduled. Fixed in three places (driver right trigger, copilot right trigger, copilot left trigger) by replacing the broken `startEnd` pattern with the command directly and using `finallyDo()` to schedule `IntakeUpCommand` on release. Also added `this` as a subsystem requirement to the first `runOnce` inside `IntakeOcilateCommand`.

---

### ~~2. `NamedCommands.registerCommand("runMotorCommand", ...)` registered twice~~ ✓ FIXED

**Fixed in:** `b230672`

The duplicate registration at line 129 overwrote the correct continuous auto-velocity registration with a one-shot fixed-speed command. Duplicate removed — autos now use the correct `Commands.run()` registration with live distance-based RPM.

---

### ~~3. `System.out.println` in `TurretSubsystem.periodic()` — runs 50x per second~~ ✓ FIXED

**Fixed in:** `2c84ea3`

Removed the `println` from `periodic()`. Turret angle is already published to SmartDashboard by the existing Notifier.

---

## HIGH — Significant risk of reduced performance

### 4. Turret aiming is commented out of both shooting triggers

**File:** `RobotContainer.java` lines 235, 294

Both `joystick.rightTrigger()` and `copilotController.rightTrigger()` have the turret aim command commented out:

```java
//turretSubsystem.setTurretPositionVariable(), // replace with zero positioning if turret aiming fails
```

During a shot, the turret is not updated. The driver must pre-aim using the right bumper (`onTrue`) before pressing the trigger. If the robot moves between bumper press and trigger press, the turret is stale. This is a significant accuracy regression, especially for moving shots.

**Fix:** Add `Commands.repeatingSequence(turretSubsystem.setTurretPositionVariable(), Commands.waitSeconds(0.5))` back into both trigger parallel groups, or use `Commands.run(turretSubsystem::turretAutoAimToHubImmediate, turretSubsystem)` for continuous tracking.

---

### 5. `"TurretAutoAimToHub"` NamedCommand aims once and finishes

**File:** `RobotContainer.java` line 128

```java
NamedCommands.registerCommand("TurretAutoAimToHub", turretSubsystem.setTurretPositionVariable());
```

`setTurretPositionVariable()` is `Commands.runOnce(...)`. In autonomous paths, this aims once and immediately finishes. PathPlanner will proceed to the next action. If the robot is still moving when the shot fires, the turret will be stale.

**Fix:** Register with a command that continuously tracks:
```java
NamedCommands.registerCommand("TurretAutoAimToHub",
    Commands.run(turretSubsystem::turretAutoAimToHubImmediate, turretSubsystem));
```

---

### 6. Default auto `"middle boring"` does not exist

**File:** `RobotContainer.java` line 133

```java
autoChooser = AutoBuilder.buildAutoChooser("middle boring");
```

No auto file named `"middle boring"` exists. Available autos are: `simple left auto`, `simple middle auto`, `simple right auto`, `ball buster left`, `ball buster right`, `left depot intake auto`, `left main feed auto`, `left main intake auto`, `right feed intake auto`, `right main feed auto`, `right main intake auto`, `royally skrew them over left auto`, `royally skrew them over right auto`, `middle depot intake auto`.

PathPlanner will log an error and fall back to the first auto alphabetically (`ball buster left`). If the driver doesn't explicitly select an auto in the chooser before the match, the robot will run the wrong routine.

**Fix:** Change the default to an existing auto name such as `"simple middle auto"`.

---

## MEDIUM — Potential issues that need attention

### 8. `System.out.println` in command execute loops

**Files:** `AlignToHub.java` line 152, `AlignToReefTagRelative.java` line 92, `FieldAreaCheckerSubsystem.java` lines 35/38

These print every loop iteration when their commands are running. Same performance concern as issue #3.

**Fix:** Remove these lines or move telemetry to SmartDashboard.

---

### 9. CAN ID 14 conflict between two subsystems

**Files:** `IntakeLevelSubsystem.java` line 26, `ShooterAimSubsystem.java` line 34

Both use `TalonFX(14)`. `ShooterAimSubsystem` is currently commented out in RobotContainer, so this doesn't bite today. However, if anyone uncomments the ShooterAim lines during a match scramble, two subsystems will fight over the same motor.

**Fix:** Assign `ShooterAimSubsystem` a unique CAN ID or remove the class entirely.

---

### 10. Intake jam detection logic is inverted

**File:** `IntakeWheelSubsystem.java` lines 86–90

```java
if (velocity.getValueAsDouble() < 0.5) {
    SmartDashboard.putBoolean("Intake Jammed", false); // ← LOW velocity = NOT jammed??
} else {
    SmartDashboard.putBoolean("Intake Jammed", true);
}
```

A velocity below 0.5 means the motor is barely moving — that is the jammed state. The boolean is backwards. The supply current check that would add a second condition is commented out.

**Fix:** Flip the condition: `< 0.5` should report `true` (jammed).

---

### 11. Two Notifiers in TurretSubsystem doing overlapping work

**File:** `TurretSubsystem.java` lines 98–103

```java
speedNotifier = new Notifier(this::updateTurretAngle);
speedNotifier2 = new Notifier(this::updateTurretAngle2);
speedNotifier.startPeriodic(0.5);
speedNotifier2.startPeriodic(0.5);
```

Two background threads both writing to SmartDashboard at 0.5s intervals. Notifiers run on separate threads — simultaneous SmartDashboard writes can cause data races. If the two methods are doing similar work, consolidate into one Notifier.

---

### 12. `Pose2d vision` field is dead code — evaluated once at construction, never read

**File:** `CommandSwerveDrivetrain.java` line 73

```java
Pose2d vision = LimelightHelpers.getBotPose2d_wpiBlue("limelight-fifteen");
```

This field is initialized once when the class is constructed and is never referenced again. It does nothing and calls the wrong Limelight method (not MegaTag2).

**Fix:** Delete this field.

---

### 13. `TestPIDMotorSubsystem` instantiated with CAN ID 999

**File:** `RobotContainer.java` line 71

```java
private final TestPIDMotorSubsystem pidcontroler = new TestPIDMotorSubsystem();
```

CAN ID 999 is a placeholder. CTRE will log an error for every periodic cycle trying to communicate with a nonexistent device, creating unnecessary CAN bus noise. `pidcontroler` is never used anywhere in RobotContainer.

**Fix:** Remove the instantiation and the class file.

---

## LOW — Code quality

### 14. Commented-out control bindings remove in-match tuning ability

**File:** `RobotContainer.java` lines 194–199

Shooter RPM increment/decrement commands are commented out. If the distance table proves wrong mid-match, there is no way to adjust RPM without redeploying code.

**Recommendation:** Bind at least one set of +/- RPM buttons (copilot D-pad suggested) before competition.

---

### 15. Large blocks of commented-out code reduce readability

Throughout `RobotContainer.java` and `TurretSubsystem.java` there are multi-line commented blocks of alternative implementations. These obscure the active logic during match-day debugging.

**Recommendation:** Delete stale alternatives and rely on git history.

---

## Summary Table

| # | Severity | Issue | File |
|---|----------|-------|------|
| 1 | ~~BLOCKER~~ ✓ | `IntakeOcilateCommand` discarded in lambda — intake never oscillates | fcb7b1d |
| 2 | ~~BLOCKER~~ ✓ | Duplicate `"runMotorCommand"` NamedCommand — autos get one-shot fixed speed | b230672 |
| 3 | ~~BLOCKER~~ ✓ | `System.out.println` in `periodic()` — 50x/sec loop pressure | 2c84ea3 |
| 4 | HIGH | Turret aiming commented out of both shooting triggers | RobotContainer.java:235,294 |
| 5 | HIGH | `TurretAutoAimToHub` NamedCommand uses `runOnce` — aims once in auto | RobotContainer.java:128 |
| 6 | HIGH | Default auto `"middle boring"` does not exist | RobotContainer.java:133 |
| 7 | HIGH | Shooter RPM table is uncalibrated placeholders | ShooterPowerSubsystem.java:188 |
| 8 | MEDIUM | `System.out.println` in command execute loops | AlignToHub.java:152, AlignToReefTagRelative.java:92 |
| 9 | MEDIUM | CAN ID 14 conflict between IntakeLevelSubsystem and ShooterAimSubsystem | IntakeLevelSubsystem.java:26 |
| 10 | MEDIUM | Intake jam detection logic is inverted | IntakeWheelSubsystem.java:86 |
| 11 | MEDIUM | Two Notifiers in TurretSubsystem doing overlapping SmartDashboard writes | TurretSubsystem.java:98 |
| 12 | MEDIUM | `Pose2d vision` field dead code — initialized once, never read | CommandSwerveDrivetrain.java:73 |
| 13 | MEDIUM | `TestPIDMotorSubsystem` (CAN ID 999) instantiated but unused | RobotContainer.java:71 |
| 14 | LOW | Shooter RPM tuning commands commented out | RobotContainer.java:194 |
| 15 | LOW | Large commented-out code blocks obscure active logic | RobotContainer, TurretSubsystem |

---

## Before Next Match — Minimum Required Fixes

1. ~~Fix `IntakeOcilateCommand` lambda — intake doesn't move during shooting (issue #1)~~ ✓
2. ~~Delete duplicate `"runMotorCommand"` registration — autos shoot wrong (issue #2)~~ ✓
3. ~~Remove `System.out.println` from `periodic()` — loop overrun risk (issue #3)~~ ✓
4. Change auto default to a real auto name (issue #6)
5. Calibrate shooter RPM table with measured data (issue #7)
