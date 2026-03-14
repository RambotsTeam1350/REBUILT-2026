# AlignToHub Command

## Overview

The `AlignToHub` command automatically aligns the turret to the hub using AprilTag detection from the Limelight camera. It uses vision data to calculate the precise angle from the turret to the hub and rotates the turret accordingly.

## Implementation Details

### Key Features

1. **AprilTag Detection** - Uses `LimelightHelpers.getRawFiducials()` to get vision data from the `limelight-fifteen` camera
2. **Target Identification** - Searches for a specific AprilTag ID to locate the hub
3. **Angle Calculation** - Uses the `txnc` (normalized horizontal offset) from the AprilTag detection:
   - Scales the normalized offset (-1 to 1) by the camera's 63.3° horizontal FOV
   - Adds this offset to the current turret position to get the target angle
4. **Turret Control** - Commands the turret via `setTurretAngle()` method using MotionMagic control
5. **Pose Estimator Fallback** - If the AprilTag is not visible, falls back to using the pose estimator-based auto-aim
6. **Alignment Verification** - Checks if the angle offset is within 2° tolerance for 5 consecutive cycles before finishing

### Constructor

```java
new AlignToHub(TurretSubsystem turretSubsystem, int hubAprilTagID)
```

**Parameters:**
- `turretSubsystem` - The turret subsystem to control
- `hubAprilTagID` - The AprilTag ID of the hub to align to

### Constants

- `ALIGNMENT_TOLERANCE_DEGREES` = 2.0° - How close the turret must be to consider aligned
- `REQUIRED_ON_TARGET_COUNT` = 5 - Number of consecutive cycles turret must be on target before finishing
- `limelightName` = "limelight-fifteen" - The Limelight camera name
- `cameraHorizontalFOV` = 63.3° - Limelight horizontal field of view

### Command Lifecycle

1. **Initialize** - Resets the consecutive on-target counter
2. **Execute** (called repeatedly):
   - Retrieves raw fiducial data from Limelight
   - Searches for the hub's AprilTag by ID
   - If found:
     - Calculates angle offset using `txnc` value
     - Commands turret to new target angle
     - Tracks if within tolerance
   - If not found:
     - Falls back to pose estimator auto-aim
     - Resets on-target counter
3. **IsFinished** - Returns true when turret has been on target for 5 consecutive cycles
4. **End** - Stops the turret if command was interrupted

### Integration with TurretSubsystem

The command uses the new `setTurretAngle(double degrees)` method added to `TurretSubsystem` (line 202), which provides direct angle control using MotionMagic voltage control.

## Usage Example

```java
// In RobotContainer.java
private final int HUB_APRILTAG_ID = 7; // Example AprilTag ID for the hub

// Bind to a button
joystick.rightBumper().whileTrue(new AlignToHub(turretSubsystem, HUB_APRILTAG_ID));
```

## Advantages Over Pose Estimator Alone

- **More accurate** when AprilTag is visible (direct vision measurement vs. estimated position)
- **Responsive** to real-time target movement or robot motion
- **Graceful degradation** to pose estimator when vision is unavailable
- **Verified alignment** through multi-cycle tolerance checking

## File Locations

- Command: [src/main/java/frc/robot/commands/AlignToHub.java](../src/main/java/frc/robot/commands/AlignToHub.java)
- Turret Subsystem: [src/main/java/frc/robot/subsystems/TurretSubsystem.java](../src/main/java/frc/robot/subsystems/TurretSubsystem.java)
