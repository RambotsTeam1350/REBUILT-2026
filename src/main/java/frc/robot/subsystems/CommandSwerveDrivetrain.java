package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
//import com.ctre.phoenix6.signals.NeutralMode;
import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstantsLokiBot.TunerSwerveDrivetrain; //This still has to be changed when you change which bot you're using
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
    private Pigeon2 pigeon = new Pigeon2(0, "rio");
    public SwerveDrivePoseEstimator poseEstimator;

    Pose2d vision = LimelightHelpers.getBotPose2d_wpiBlue("limelight-fifteen");

    private double initialLeftDistance = 0.0;
    private double initialRightDistance = 0.0;

    //public double positiveXDistance = poseEstimator.getEstimatedPosition().getX();
    //public double positiveYDistance = poseEstimator.getEstimatedPosition().getY();
    //public double positiveRotation = poseEstimator.getEstimatedPosition().getRotation().getDegrees();



//positions of each swere module from the center of the bot
private static final Translation2d frontLeftLocation = new Translation2d(-0.29,0.29); // 2 numbers will go here, x and y position for each one. From the center of the bot.
private static final Translation2d frontRightLocation = new Translation2d(0.29,0.29); // so like actual numbers gotta go in these translation2d objects
private static final Translation2d backLeftLocation = new Translation2d(-0.29,-0.29); //Meters
private static final Translation2d backRightLocation = new Translation2d(0.29,-0.29); // 0.58 left to right, square

private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation,
    frontRightLocation,
    backLeftLocation,
    backRightLocation
);

// Do NOT cache module positions here. We must read the current module positions
// each update so the pose estimator can integrate wheel movement.
private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        getModule(0).getCachedPosition(),
        getModule(1).getCachedPosition(),
        getModule(2).getCachedPosition(),
        getModule(3).getCachedPosition(),
    };
}


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

    // Limelight orientation call moved to constructor after poseEstimator is initialized.
/* private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDeadband(0.1) // Add a joystick deadband
    .withDriveRequestType(SwerveAppData.DriveRequestType.kOpenLoopForward); // Or other drive types
*/
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;  

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
       
        
        // initial left/right wheel distances (meters)

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroscopeRotation(),
            getModulePositions(),
            new Pose2d(0, 0, new Rotation2d())
        );

        // Initialize Limelight robot orientation now that poseEstimator exists
        LimelightHelpers.SetRobotOrientation(
            "limelight-fifteen",
            poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );

          LimelightHelpers.SetRobotOrientation(
            "limelight-three",
            poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
             0, 0, 0, 0, 0
        ); 

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
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
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
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
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
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

           
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

          
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
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
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        } 

        // Read fresh module positions each loop so odometry integrates wheel motion
        SwerveModulePosition[] modulePositions = getModulePositions();
        poseEstimator.update(
            getGyroscopeRotation(),
            modulePositions
        );

    // MegaTag2 requires updated robot orientation EVERY cycle for accurate pose estimates.
    double currentHeading = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation("limelight-fifteen", currentHeading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-three", currentHeading, 0, 0, 0, 0, 0);

    frc.robot.LimelightHelpers.PoseEstimate llEstimate5 =
    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-fifteen");

    frc.robot.LimelightHelpers.PoseEstimate llEstimate3 =
    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-three");

        // Maximum tag distance beyond which pose estimates are too noisy to be useful.
        final double kMaxTagDistanceMeters = 5.0;

        if (LimelightHelpers.validPoseEstimate(llEstimate5)
                && llEstimate5.tagCount >= 1
                && llEstimate5.avgTagDist < kMaxTagDistanceMeters) {
            // Scale X/Y stdDevs by distance squared: close tags get high trust, far tags
            // get low trust. At 1m with 2 tags → 0.1; at 2m → 0.4; at 4m → 1.6.
            // High rotation stddev keeps heading governed by Pigeon2, not Limelight.
            double xyStdDev = (llEstimate5.tagCount >= 2)
                    ? 0.1 * llEstimate5.avgTagDist * llEstimate5.avgTagDist
                    : 0.3 * llEstimate5.avgTagDist * llEstimate5.avgTagDist;
            Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, Math.toRadians(9999));
            addVisionMeasurement(llEstimate5.pose, llEstimate5.timestampSeconds, stdDevs);
        }

        if (LimelightHelpers.validPoseEstimate(llEstimate3)
                && llEstimate3.tagCount >= 1
                && llEstimate3.avgTagDist < kMaxTagDistanceMeters) {
            double xyStdDev = (llEstimate3.tagCount >= 2)
                    ? 0.1 * llEstimate3.avgTagDist * llEstimate3.avgTagDist
                    : 0.3 * llEstimate3.avgTagDist * llEstimate3.avgTagDist;
            Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, Math.toRadians(9999));
            addVisionMeasurement(llEstimate3.pose, llEstimate3.timestampSeconds, stdDevs);
        }
      System.out.println("X: " + poseEstimator.getEstimatedPosition().getX() + " Y: " + poseEstimator.getEstimatedPosition().getY() + " Angle: " + poseEstimator.getEstimatedPosition().getRotation().getDegrees() + " degrees");
         
    }

    /**
     * Gets the current robot rotation from the gyroscope.
     *
     * @return The robot rotation as a Rotation2d.
     */
    
    public Rotation2d getGyroscopeRotation() {
        //π
        // Replace `gyro` with the actual gyroscope object in your code
        // pigeon.getYaw() returns a StatusSignal<Angle> in Phoenix 6.
        // Extract the Angle with getValue() and use the radians accessor (getRadians()).
        // Pigeon2 in Phoenix 6 exposes a direct Rotation2d accessor.
        // Use that instead of working with StatusSignal/Angle wrappers.
        return pigeon.getRotation2d();
    }

    /**
     * Expose the internal WPILib pose estimator so other subsystems (e.g. turret)
     * can use the robot pose for calculations.
     *
     * @return the SwerveDrivePoseEstimator used by this drivetrain
     */
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

/**
 * Moves the swerve drive at a specific speed in a given direction, with an option for field-relative movement.
 *
 * @param speedMetersPerSecond The desired speed in meters per second.
 * @param direction The desired direction of movement as a Rotation2d.
 * @param isFieldRelative Whether the movement should be field-relative.
 */
public void drive(Translation2d translation, double directionDegrees, boolean isFieldRelative) {
    
    Rotation2d direction = Rotation2d.fromDegrees(directionDegrees);

    Translation2d adjustedTranslation = new Translation2d(
        translation.getNorm() * direction.getCos(),
        translation.getNorm() * direction.getSin()
    );


    // Get the current robot orientation
    Rotation2d robotAngle = getGyroscopeRotation();

    // Calculate the desired chassis speeds
    ChassisSpeeds chassisSpeeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), 0.0, robotAngle)
        : new ChassisSpeeds(translation.getX(), translation.getY(), 0.0);

    // Convert chassis speeds to swerve module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Normalize wheel speeds to prevent saturation
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeedMetersPerSecond);

    // Set the desired states to the swerve modules
    /* 
    for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].setDesiredState(moduleStates[i]);
    }*/
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
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    //@Override
    /*    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        double fpgaTime = Utils.fpgaToCurrentTime(timestampSeconds);
        // Forward to the underlying CTRE drivetrain (it will handle fusion internally)
        super.addVisionMeasurement(visionRobotPoseMeters, fpgaTime);
        // Also forward to our local WPILib pose estimator (if present)
        if (poseEstimator != null) {
            poseEstimator.addVisionMeasurement(visionRobotPoseMeters, fpgaTime);
        }
    } */

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        double fpgaTime = Utils.fpgaToCurrentTime(timestampSeconds);
        super.addVisionMeasurement(visionRobotPoseMeters, fpgaTime, visionMeasurementStdDevs);
        if (poseEstimator != null) {
            // WPILib estimator supports passing measurement standard deviations via setVisionMeasurementStdDevs
            poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
            poseEstimator.addVisionMeasurement(visionRobotPoseMeters, fpgaTime);
        }
    }
}
