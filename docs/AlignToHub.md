# AlignToHub Command

## Overview

The `AlignToHub` command automatically aligns the turret to the hub using AprilTag detection from the Limelight camera. It uses vision data to calculate the precise angle from the turret to the hub and rotates the turret accordingly, **accounting for the geometric offset between the camera and turret positions on the robot**.

## Implementation Details

### Key Features

1. **AprilTag Detection** - Uses `LimelightHelpers.getRawFiducials()` to get vision data from the `limelight-fifteen` camera
2. **Target Identification** - Searches for a specific AprilTag ID to locate the hub
3. **Angle Calculation with Camera-Turret Offset Compensation**:
   - Scales the `txnc` (normalized horizontal offset -1 to 1) by the camera's 63.3° horizontal FOV to get the angle from the camera's perspective
   - Calculates the geometric angular offset between camera and turret positions on the robot
   - Uses the target distance from the AprilTag to compute the parallax correction
   - Combines both angles to determine the true turret angle needed for alignment
4. **Turret Control** - Commands the turret via `setTurretAngle()` method using MotionMagic control
5. **Pose Estimator Fallback** - If the AprilTag is not visible, falls back to using the pose estimator-based auto-aim
6. **Alignment Verification** - Checks if the total angle correction is within 2° tolerance for 5 consecutive cycles before finishing

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
     - Calculates `angleToTagFromCamera` using `txnc` value scaled by FOV
     - Gets camera and turret positions on the robot
     - Calculates `geometricAngleOffset` based on camera-turret offset and target distance
     - Computes `totalAngleCorrection = angleToTagFromCamera + geometricAngleOffset`
     - Commands turret to `currentTurretAngle + totalAngleCorrection`
     - Tracks if within tolerance
   - If not found:
     - Falls back to pose estimator auto-aim
     - Resets on-target counter
3. **IsFinished** - Returns true when turret has been on target for 5 consecutive cycles
4. **End** - Stops the turret if command was interrupted

### Camera-Turret Offset Geometry

The command accounts for the physical separation between the Limelight camera and the turret on the robot:

- **Camera Position**: Defined in `TurretSubsystem.XofCameraOnBot` and `YofCameraOnBot` (relative to robot center)
- **Turret Position**: Defined in `TurretSubsystem.XofTurretOnBot` and `YofTurretOnBot` (relative to robot center)
- **Geometric Correction**: Calculates the perpendicular offset between camera line-of-sight and turret position, then uses `atan2(offset, distance)` to find the angular correction needed

This ensures the turret aims accurately even though the camera is mounted elsewhere on the robot.

### Integration with TurretSubsystem

The command uses these methods from `TurretSubsystem`:
- `setTurretAngle(double degrees)` (line 206) - Direct angle control using MotionMagic voltage control
- `getCameraPositionOnBot()` (line 215) - Returns camera position on robot
- `getTurretPositionOnBot()` (line 224) - Returns turret position on robot
- `getTurretRotation()` (line 123) - Gets current turret angle in degrees

## Configuration Required

Before using this command, you **must** measure and configure the camera position on the robot in `TurretSubsystem.java`:

```java
// In TurretSubsystem.java lines 58-59
public double XofCameraOnBot = 0.0; // TODO: Measure actual camera X offset (meters)
public double YofCameraOnBot = 0.0; // TODO: Measure actual camera Y offset (meters)
```

Measure from the robot center to the Limelight camera mount position in the robot's coordinate frame (X = forward, Y = left).

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
