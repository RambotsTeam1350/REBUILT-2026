// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ThroatAndIndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Intake.IntakeLevelSubsystem;
import frc.robot.subsystems.Intake.IntakeWheelSubsystem;
import frc.robot.subsystems.Shooter.ShooterPowerSubsystem;
import frc.robot.subsystems.TestPIDMotorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final IntakeLevelSubsystem intaketestSubsytem = new IntakeLevelSubsystem();
    private final IntakeWheelSubsystem IntakeWheelSubsystem = new IntakeWheelSubsystem();
    private final ShooterPowerSubsystem ShooterSubsystem = new ShooterPowerSubsystem();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final ThroatAndIndexerSubsystem ThroatAndIndexerSubsystem = new ThroatAndIndexerSubsystem();
    private final TestPIDMotorSubsystem pidcontroler = new TestPIDMotorSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final TurretSubsystem turretSubsystem;
    private final SendableChooser<Command> autoChooser;
    
    //private final SparkFlex motor = new SparkFlex(16, MotorType.kBrushless);

    public RobotContainer() {
        // Explicitly ensure the drivetrain configures AutoBuilder before attempting
        // to build the chooser. If configuration fails, fall back to an empty chooser
        // to avoid crashing the robot code.

        // Construct the turret after the drivetrain so we can pass the drivetrain's
        // pose estimator into the turret constructor.
        turretSubsystem = new TurretSubsystem(drivetrain.getPoseEstimator());

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("blue middle");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // joystick.rightBumper().onTrue(new AlignToReefTagRelative(true, drivetrain));

        //joystick.b().onTrue(intaketestSubsytem.IntakeDownCommand());
        //joystick.b().onTrue(pidcontroler.MotorTest());
        //joystick.a().onTrue(pidcontroler.StopMotorCommand());
        //joystick.b().onTrue(turretSubsystem.TurretTestSpeed());
        joystick.a().onTrue(turretSubsystem.TurretAutoAimToHub());

        joystick.x().onTrue(turretSubsystem.TurretToMaxPosition());
    
        //joystick.y().onTrue(turretSubsystem.TurretTestSpeed());
        joystick.b().onTrue(turretSubsystem.TurretToZero());
        
        //joystick.a().onTrue(intaketestSubsytem.IntakeUpCommand());
        //joystick.x().onTrue(IntakeWheelSubsystem.runMotorCommand());

        //joystick.y().onTrue(IntakeWheelSubsystem.stopMotorCommand());
        //joystick.b().onTrue(intaketestSubsytem.IntakeDownCommand());

       // joystick.x().onTrue(ShooterSubsystem.runMotorCommand());
       // joystick.y().onTrue(ShooterSubsystem.stopMotorCommand());

       // joystick.povUp().onTrue(climberSubsystem.ClimbUpCommand());
       // joystick.povDown().onTrue(climberSubsystem.ClimbDownCommand());

        // joystick.x().onTrue(pidcontroler.MotionMagicCommand());
        // joystick.y().onTrue(pidcontroler.StopMotionMagicCommand());

        // SmartDashboard.putData(autochooser);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}