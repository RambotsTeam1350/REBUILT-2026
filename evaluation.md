# Competition Code Evaluation — FRC Team 1350 La Salle Robotics 2026

**Evaluated:** 2026-04-15  
**Branch:** aiming-fixes

Issues are grouped by severity. Fix blockers before any match. High severity items should be addressed before eliminations.

---

## BLOCKER — Will cause wrong behavior in competition

### 1. `IntakeOcilateCommand()` is silently discarded — intake never oscillates

**Files:** `RobotContainer.java` lines 249, 308, 334

In three separate shooting commands (driver right trigger, copilot right trigger, copilot left trigger), `IntakeOcilateCommand()` is called inside a void lambda:

```java
Commands.startEnd(
    () -> {
        intaketestSubsystem.IntakeOcilateCommand(); // ← returns Command, discarded immediately
    },
    () -> intaketestSubsystem.IntakeUpCommand(),    // ← also discarded
    intaketestSubsystem)
```

`IntakeOcilateCommand()` builds and returns a `Command` object using `Commands.sequence(...).repeatedly()`. Calling it inside a lambda that returns `void` creates the Command object and immediately throws it away — it is never scheduled. The intake motor receives no new commands during shooting. It holds whatever position it was in last.

The end lambda has the same problem — `IntakeUpCommand()` is also discarded, so the intake does not return up on trigger release from within this command.

**Fix:** Use `Commands.run(() -> intaketestSubsystem.setIntakePosition(-7.0), intaketestSubsystem)` for a continuous hold, or rework `IntakeOcilateCommand` to use imperative calls instead of building a command. Using a `Commands.repeatingSequence` with direct motor calls is the cleanest approach.

---

### 2. `NamedCommands.registerCommand("runMotorCommand", ...)` registered twice — second overwrites first

**File:** `RobotContainer.java` lines 110–119 and 129–130

```java
// Line 110 — first registration: continuous auto-velocity
NamedCommands.registerCommand("runMotorCommand",
    Commands.parallel(
        ThroatAndIndexerSubsystem.runMotorCommand(),
        Commands.run(
            () -> ShooterSubsystem.runShooterWithAutoVelocity(turretSubsystem.getDistanceToHub()),
            turretSubsystem)));

// ...

// Line 129 — second registration: overwrites with one-shot fixed speed
NamedCommands.registerCommand("runMotorCommand",
    Commands.parallel(ShooterSubsystem.runMotorCommand(), ThroatAndIndexerSubsystem.runMotorCommand()));
```

The second registration wins. `ShooterSubsystem.runMotorCommand()` is `Commands.runOnce(this::runShooter)` — it fires once at a fixed speed and immediately finishes. Every autonomous path that calls `"runMotorCommand"` gets a one-shot, fixed-speed command instead of the intended continuous auto-velocity control. PathPlanner will continue the path the moment this command finishes, likely before any ball leaves the robot.

**Fix:** Delete the duplicate registration at lines 129–130.

---

### 3. `System.out.println` in `TurretSubsystem.periodic()` — runs 50x per second

**File:** `TurretSubsystem.java` line 118

```java
System.out.println("turret angle: " + motorPosition.getValueAsDouble());
```

`periodic()` runs every 20ms. This will produce ~50 log lines per second, filling the console buffer and creating measurable CPU pressure in the robot loop. On a busy match day with poor radio, this can cause loop overruns that DS will flag.

**Fix:** Remove this line or push to SmartDashboard via the existing Notifier at 0.5 Hz.

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

### 7. Shooter RPM interpolation table is not calibrated

**File:** `ShooterPowerSubsystem.java` lines 188–190

```java
double[] shooterRPMPoints = { 1300.00, 1400.00, 1500.00, 1600.00, 1700.00, 1800.00, 1900.00, 2000.00, 2100.00, 2200.00, 2400.00 };
```

The table has a TODO comment. Values are linearly spaced placeholders, not measured shot data. Any command using `runShooterWithAutoVelocity()` — including the copilot right trigger and the driver `povUp` sequence — will use these uncalibrated values.

**Fix:** Measure and populate with actual (distance → RPM) pairs across the expected shooting range before competition.

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
| 1 | BLOCKER | `IntakeOcilateCommand` discarded in lambda — intake never oscillates | RobotContainer.java:249,308,334 |
| 2 | BLOCKER | Duplicate `"runMotorCommand"` NamedCommand — autos get one-shot fixed speed | RobotContainer.java:129 |
| 3 | BLOCKER | `System.out.println` in `periodic()` — 50x/sec loop pressure | TurretSubsystem.java:118 |
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

1. Fix `IntakeOcilateCommand` lambda — intake doesn't move during shooting (issue #1)
2. Delete duplicate `"runMotorCommand"` registration — autos shoot wrong (issue #2)
3. Remove `System.out.println` from `periodic()` — loop overrun risk (issue #3)
4. Change auto default to a real auto name (issue #6)
5. Calibrate shooter RPM table with measured data (issue #7)
