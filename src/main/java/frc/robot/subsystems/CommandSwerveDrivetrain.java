package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
//import com.ctre.phoenix6.signals.NeutralMode;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstantsLokiBotDistricts.TunerSwerveDrivetrain; //This still has to be changed when you change which bot you're using
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private double maxSpeedMetersPerSecond = 5.0; // Example max speed, adjust as needed
    private final Field2d m_wpiLibField = new Field2d();
    // False until the first valid vision measurement is accepted. While false, the
    // pose jump filter is bypassed so an AprilTag can seed the initial position.
    private boolean hasReceivedVisionFix = false;
    // Counts consecutive frames where all vision measurements were rejected by the
    // jump filter. When it hits the threshold the flag is cleared so a 2-tag
    // detection can re-seed the estimator and recover from a bad initial pose.
    private int consecutiveVisionRejections = 0;
    private static final int kVisionRecoveryThreshold = 30; // ~0.6 s at 50 Hz

    // public double positiveXDistance =
    // poseEstimator.getEstimatedPosition().getX();
    // public double positiveYDistance =
    // poseEstimator.getEstimatedPosition().getY();
    // public double positiveRotation =
    // poseEstimator.getEstimatedPosition().getRotation().getDegrees();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // Limelight orientation call moved to constructor after poseEstimator is
    // initialized.
    /*
     * private final SwerveRequest.FieldCentric driveRequest = new
     * SwerveRequest.FieldCentric()
     * .withDeadband(0.1) // Add a joystick deadband
     * .withDriveRequestType(SwerveAppData.DriveRequestType.kOpenLoopForward); // Or
     * other drive types
     */
    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {

        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        SmartDashboard.putData("WPILib Field", m_wpiLibField);

        // Robot starts at heading 0 — orientation will be updated every loop.
        LimelightHelpers.SetRobotOrientation("limelight-fifteen", 0, 0, 0, 0, 0, 0);
        // LimelightHelpers.SetRobotOrientation("limelight-three", 0, 0, 0, 0, 0, 0);

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            DriverStation.reportError("Failed to load RobotConfig", e.getStackTrace());
            throw new RuntimeException("RobotConfig load failed", e);

        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> getState().Pose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setControl(
                        m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), // Method
                                                                                                           // that will
                                                                                                           // drive the
                                                                                                           // robot
                                                                                                           // given
                                                                                                           // ROBOT
                                                                                                           // RELATIVE
                                                                                                           // ChassisSpeeds.
                                                                                                           // Also
                                                                                                           // optionally
                                                                                                           // outputs
                                                                                                           // individual
                                                                                                           // module
                                                                                                           // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        // Ensure the PathPlanner AutoBuilder is configured for this drivetrain.
        // TunerConstants.createDrivetrain() uses this constructor, so configure
        // AutoBuilder here so callers (such as RobotContainer) can safely call
        // AutoBuilder.buildAutoChooser(...) afterward.
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        m_wpiLibField.setRobotPose(getState().Pose);

        // MegaTag2 requires updated robot orientation EVERY cycle. Use the CTRE fused
        // pose heading (not raw Pigeon2 yaw) so field-forward is correct regardless of
        // boot orientation. Rotation stddev is effectively infinite so Limelight cannot
        // feed back into the heading, preventing a correction loop.
        double currentHeading = getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation("limelight-fifteen", currentHeading, 0, 0, 0, 0, 0);
        // LimelightHelpers.SetRobotOrientation("limelight-three", currentHeading, 0, 0, 0, 0, 0);

        frc.robot.LimelightHelpers.PoseEstimate llEstimate5 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-fifteen");

        frc.robot.LimelightHelpers.PoseEstimate llEstimate3 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-three");

        // Maximum tag distance beyond which pose estimates are too noisy to be useful.
        final double kMaxTagDistanceMeters = 5.0;
        // Jump filter threshold — reject measurements that imply the robot teleported.
        // Bypassed until the first valid fix is accepted (hasReceivedVisionFix ==
        // false),
        // so an AprilTag can seed the initial pose from origin during off-field testing
        // or when no PathPlanner auto starting pose is set.
        final double kMaxPoseJumpMeters = 1.0;
        Pose2d currentPose = getState().Pose;

        if (LimelightHelpers.validPoseEstimate(llEstimate5)
                && llEstimate5.tagCount >= 1
                && llEstimate5.avgTagDist < kMaxTagDistanceMeters) {
            double xyStdDev5 = (llEstimate5.tagCount >= 2)
                    ? 0.1 * llEstimate5.avgTagDist * llEstimate5.avgTagDist
                    : 0.3 * llEstimate5.avgTagDist * llEstimate5.avgTagDist;
            if (!hasReceivedVisionFix && llEstimate5.tagCount >= 2) {
                // Require 2+ tags for hard-seed: single-tag pose ambiguity is unreliable
                // and can lock the estimator into a wrong position.
                resetPose(llEstimate5.pose);
            } else if (hasReceivedVisionFix && llEstimate5.pose.getTranslation()
                    .getDistance(currentPose.getTranslation()) < kMaxPoseJumpMeters) {
                consecutiveVisionRejections = 0;
                addVisionMeasurement(llEstimate5.pose, llEstimate5.timestampSeconds,
                        VecBuilder.fill(xyStdDev5, xyStdDev5, Math.toRadians(9999)));
            } else if (hasReceivedVisionFix) {
                consecutiveVisionRejections++;
            } else {
                // Single tag, no fix yet — soft-seed with high stdDev to nudge from origin
                // without committing to a potentially ambiguous pose.
                addVisionMeasurement(llEstimate5.pose, llEstimate5.timestampSeconds,
                        VecBuilder.fill(xyStdDev5 * 3, xyStdDev5 * 3, Math.toRadians(9999)));
            }
        }

        if (LimelightHelpers.validPoseEstimate(llEstimate3)
                && llEstimate3.tagCount >= 1
                && llEstimate3.avgTagDist < kMaxTagDistanceMeters) {
            double xyStdDev3 = (llEstimate3.tagCount >= 2)
                    ? 0.1 * llEstimate3.avgTagDist * llEstimate3.avgTagDist
                    : 0.3 * llEstimate3.avgTagDist * llEstimate3.avgTagDist;
            if (!hasReceivedVisionFix && llEstimate3.tagCount >= 2) {
                resetPose(llEstimate3.pose);
            } else if (hasReceivedVisionFix && llEstimate3.pose.getTranslation()
                    .getDistance(currentPose.getTranslation()) < kMaxPoseJumpMeters) {
                consecutiveVisionRejections = 0;
                addVisionMeasurement(llEstimate3.pose, llEstimate3.timestampSeconds,
                        VecBuilder.fill(xyStdDev3, xyStdDev3, Math.toRadians(9999)));
            } else if (hasReceivedVisionFix) {
                consecutiveVisionRejections++;
            } else {
                addVisionMeasurement(llEstimate3.pose, llEstimate3.timestampSeconds,
                        VecBuilder.fill(xyStdDev3 * 3, xyStdDev3 * 3, Math.toRadians(9999)));
            }
        }

        // If both cameras have been consistently rejected by the jump filter for too
        // long, the initial seed was likely bad. Clear the flag so a reliable 2-tag
        // detection can re-seed the estimator.
        if (consecutiveVisionRejections >= kVisionRecoveryThreshold) {
            hasReceivedVisionFix = false;
            consecutiveVisionRejections = 0;
        }
        // System.out.println("X: " + poseEstimator.getEstimatedPosition().getX() + " Y:
        // " + poseEstimator.getEstimatedPosition().getY() + " Angle: " +
        // poseEstimator.getEstimatedPosition().getRotation().getDegrees() + "
        // degrees");

        //System.out.println(position);

    }

    /**
     * Gets the current robot rotation from the gyroscope.
     *
     * @return The robot rotation as a Rotation2d.
     */

    public Rotation2d getGyroscopeRotation() {
        // π
        // Replace `gyro` with the actual gyroscope object in your code
        // pigeon.getYaw() returns a StatusSignal<Angle> in Phoenix 6.
        // Extract the Angle with getValue() and use the radians accessor
        // (getRadians()).
        // Pigeon2 in Phoenix 6 exposes a direct Rotation2d accessor.
        // Use that instead of working with StatusSignal/Angle wrappers.
        return getPigeon2().getRotation2d();
    }

    /**
     * Returns the current robot pose from the CTRE fused estimator (250 Hz).
     * Use this instead of a separate WPILib estimator so all subsystems share one
     * source of truth.
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * Resets the robot pose. Called by PathPlanner when an auto routine has a
     * defined starting pose. Marking hasReceivedVisionFix=true here means the
     * jump filter is active immediately — we already know where we are.
     */
    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
        hasReceivedVisionFix = true;
        consecutiveVisionRejections = 0;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    // @Override
    /*
     * public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double
     * timestampSeconds) {
     * double fpgaTime = Utils.fpgaToCurrentTime(timestampSeconds);
     * // Forward to the underlying CTRE drivetrain (it will handle fusion
     * internally)
     * super.addVisionMeasurement(visionRobotPoseMeters, fpgaTime);
     * // Also forward to our local WPILib pose estimator (if present)
     * if (poseEstimator != null) {
     * poseEstimator.addVisionMeasurement(visionRobotPoseMeters, fpgaTime);
     * }
     * }
     */

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        // CTRE's addVisionMeasurement expects time in the CTRE domain.
        super.addVisionMeasurement(visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}
