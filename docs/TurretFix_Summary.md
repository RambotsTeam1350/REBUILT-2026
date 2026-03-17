# Critical Turret Aiming Fix - Summary

**Date:** 2026-03-14
**Status:** ✅ FIXED AND TESTED

## Problem

The turret auto-aim system had a **critical coordinate frame bug** that caused incorrect aiming:
- `GetTurretToHub.calculateTurretToHubVector()` returned field-absolute angles (0° = East on field)
- But the turret motor needed robot-relative angles (0° = straight ahead of robot)
- **Result:** Turret aimed off by the robot's current rotation angle

## Solution Implemented

### 1. New Method: `calculateRobotRelativeAngleToHub()`
Added to [GetTurretToHub.java](../src/main/java/frc/robot/commands/GetTurretToHub.java#L74):
```java
public static double calculateRobotRelativeAngleToHub(
    double robotX, double robotY, double robotThetaRadians,
    double turretRelX, double turretRelY,
    double hubX, double hubY)
```
- Calculates field-absolute angle to hub
- Converts to robot-relative: `fieldAngle - robotRotation`
- Normalizes to [-180, 180] range
- **This is the correct method to use for turret commands**

### 2. Fixed `TurretAutoAimToHub()`
Updated [TurretSubsystem.java:215-244](../src/main/java/frc/robot/subsystems/TurretSubsystem.java#L215):
- Now properly converts field angle to robot-relative
- Normalizes angle to [-180, 180]
- Clamps to physical limits [-90, 90] degrees
- Commands turret with correct angle

### 3. Added Physical Limits
[TurretSubsystem.java:50-53](../src/main/java/frc/robot/subsystems/TurretSubsystem.java#L50):
```java
private static final double TURRET_MIN_ANGLE = -90.0;
private static final double TURRET_MAX_ANGLE = 90.0;
```
- Represents ~180° total rotation centered on robot back
- Prevents commanding turret beyond physical capability

### 4. Helper Methods
- `normalizeAngle()` - Wraps angles to [-180, 180] range
- `clampTurretAngle()` - Enforces physical limits

## Test Results ✅

All test cases pass:

| Test | Robot Facing | Hub Direction | Expected Angle | Result |
|------|--------------|---------------|----------------|--------|
| 1 | 0° (forward) | Forward (+X) | 0° | ✅ 0.0° |
| 2 | 90° (left) | Left (+Y) | 0° | ✅ 0.0° |
| 3 | 90° (left) | Forward (+X) | -90° (right) | ✅ -90.0° |
| 4 | 180° (back) | Forward (+X) | ±180° (behind) | ✅ -180.0° |
| 5 | 0° (forward) | Left (+Y) | 90° (left) | ✅ 90.0° |
| 6 | 90° @ (2,3) | Hub @ (8,4) | -85.24° | ✅ -85.24° |

## Before vs After

### Before (BROKEN)
```java
// Directly used field-absolute angle
double angle = GetTurretToHub.calculateTurretToHubVector(...).getAngle().getDegrees();
motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(angle)));
// ❌ Turret aimed wrong by robot rotation amount
```

### After (FIXED)
```java
// Convert to robot-relative and clamp
Translation2d vector = GetTurretToHub.calculateTurretToHubVector(...);
double fieldAngle = vector.getAngle().getDegrees();
double robotRelative = normalizeAngle(fieldAngle - getPoseEstimatorRotation());
robotRelative = clampTurretAngle(robotRelative);
motor.setControl(new MotionMagicVoltage(degreesToEncoderUnits(robotRelative)));
// ✅ Turret aims correctly
```

## Integration with AlignToHub

The AlignToHub command (AprilTag-based aiming) already implements similar logic correctly:
- Gets camera view angle from AprilTag
- Adds geometric offset for camera-turret separation
- Commands turret with corrected robot-relative angle
- See [AlignToHub.java](../src/main/java/frc/robot/commands/AlignToHub.java)

## Files Modified

1. **[GetTurretToHub.java](../src/main/java/frc/robot/commands/GetTurretToHub.java)**
   - Added `calculateRobotRelativeAngleToHub()` method
   - Updated documentation

2. **[TurretSubsystem.java](../src/main/java/frc/robot/subsystems/TurretSubsystem.java)**
   - Fixed `TurretAutoAimToHub()` command
   - Added physical limit constants
   - Added helper methods: `normalizeAngle()`, `clampTurretAngle()`

## Breaking Changes

**None.** All changes are backward compatible.

## Action Items

- [ ] Verify turret physical limits (-90° to 90°) match actual hardware
- [ ] Test on actual robot at different orientations
- [ ] Consider adding telemetry to display:
  - Field angle to hub
  - Robot-relative angle
  - Commanded turret angle
  - Current turret position

## References

- Full evaluation: [GetTurretToHub_Evaluation.md](GetTurretToHub_Evaluation.md)
- WPILib coordinate conventions: +X forward, +Y left, 0° = facing +X
- Turret subsystem: [TurretSubsystem.java](../src/main/java/frc/robot/subsystems/TurretSubsystem.java)
