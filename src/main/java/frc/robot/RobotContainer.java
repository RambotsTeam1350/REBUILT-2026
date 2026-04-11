// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.commands.AlignToHub;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ThroatAndIndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Intake.IntakeLevelSubsystem;
import frc.robot.subsystems.Intake.IntakeWheelSubsystem;
//import frc.robot.subsystems.Shooter.ShooterAimSubsystem;
import frc.robot.subsystems.Shooter.ShooterPowerSubsystem;
import frc.robot.subsystems.TestPIDMotorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.AlignToHub;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.8; // 80% of top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.8; // 80% max angular rate
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final IntakeLevelSubsystem intaketestSubsystem = new IntakeLevelSubsystem();
    private final IntakeWheelSubsystem intakeWheelSubsystem = new IntakeWheelSubsystem();
    private final ShooterPowerSubsystem ShooterSubsystem = new ShooterPowerSubsystem();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController copilotController = new CommandXboxController(1);
    private final ThroatAndIndexerSubsystem ThroatAndIndexerSubsystem = new ThroatAndIndexerSubsystem();
    private final TestPIDMotorSubsystem pidcontroler = new TestPIDMotorSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final TurretSubsystem turretSubsystem;
    //private final ShooterAimSubsystem shooterAimSubsystem;
    private final SendableChooser<Command> autoChooser;
    private LimelightTarget_Detector limelight = new LimelightTarget_Detector();
    // Notifier to update the dashboard match-time widget periodically
    private final Notifier matchTimeNotifier;
    
    //private final SparkFlex motor = new SparkFlex(16, MotorType.kBrushless);

    public RobotContainer() {
        // Explicitly ensure the drivetrain configures AutoBuilder before attempting
        // to build the chooser. If configuration fails, fall back to an empty chooser
        // to avoid crashing the robot code.

        // Construct the turret after the drivetrain so we can pass the drivetrain's
        // pose estimator into the turret constructor.
        turretSubsystem = new TurretSubsystem(drivetrain.getPoseEstimator());
        // ShooterAimSubsystem uses TurretSubsystem as its distance source so both
        // subsystems share the same turret-corrected distance to hub.
            //shooterAimSubsystem = new ShooterAimSubsystem(turretSubsystem);

    configureBindings();

    //////////////////////////////////////////
    /// Path planner autos
    /// 
    

    NamedCommands.registerCommand("runIntakeMotor", intakeWheelSubsystem.runMotorCommand());
    NamedCommands.registerCommand("stopIntakeMotor", intakeWheelSubsystem.stopMotorCommand());
    NamedCommands.registerCommand("IntakeDownCommand", intaketestSubsystem.IntakeDownCommand());
    NamedCommands.registerCommand("IntakeUpCommand", intaketestSubsystem.IntakeUpCommand());
    NamedCommands.registerCommand(
        "stopMotorCommand",
        Commands.parallel(
            ThroatAndIndexerSubsystem.stopMotorCommand(),
            ShooterSubsystem.stopMotorCommand()
        )
    );
    NamedCommands.registerCommand(
        "runMotorCommand",
        Commands.parallel(
            ThroatAndIndexerSubsystem.runMotorCommand(),
            ShooterSubsystem.runMotorCommand()
            )
    );  
    ///////////////////////////////////////////////////////////////

    // Start a background notifier to update the match time on the dashboard
    matchTimeNotifier = new Notifier(this::updateMatchTimeOnDashboard);
    // update twice per second
    matchTimeNotifier.startPeriodic(0.5);



        NamedCommands.registerCommand("TurretAutoAimToHub", turretSubsystem.TurretAutoAimToHub());
        NamedCommands.registerCommand("runMotorCommand",
                Commands.parallel(ShooterSubsystem.runMotorCommand(), ThroatAndIndexerSubsystem.runMotorCommand()));
        NamedCommands.registerCommand("ClimbDownCommand", climberSubsystem.ClimbDownCommand());

        autoChooser = AutoBuilder.buildAutoChooser("middle boring");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putNumber("lower motor speed", ShooterSubsystem.lowerWheelSpeed);
        SmartDashboard.putNumber("backspin motor speed", ShooterSubsystem.backspinWheelSpeed);
    }

    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically.
                // Squared input curve: preserves sign but squares magnitude for finer low-speed control.
                drivetrain.applyRequest(() -> {
                    double vx = -joystick.getLeftY();
                    double vy = -joystick.getLeftX();
                    double rot = -joystick.getRightX();
                    return drive
                            .withVelocityX(Math.copySign(vx * vx, vx) * MaxSpeed)
                            .withVelocityY(Math.copySign(vy * vy, vy) * MaxSpeed)
                            .withRotationalRate(Math.copySign(rot * rot, rot) * MaxAngularRate);
                }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

      //  joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        /*  joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
*/
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

     
        
     
        //joystick.rightBumper().whileTrue(new AlignToHub(drivetrain, limelight, 0)); // Align to hub with no offset

//////////////////////////////////////////////////////////////////////////////
/// DRIVER CONTROLS
//////////////////////////////////////////////////////////////////////////////

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.a().whileTrue(turretSubsystem.aimAtHubViaPose());    // Aim via pose estimator
        //joystick.b().whileTrue(turretSubsystem.aimAtHubViaVision());  // Aim via Limelight vision
        //joystick.a().onTrue(turretSubsystem.TurretToZero());

        joystick.x().onTrue(ShooterSubsystem.decreaseBackspinWheelSpeed());
        joystick.y().onTrue(ShooterSubsystem.increaseBackspinWheelSpeed());
        joystick.a().onTrue(ShooterSubsystem.decreaseLowerWheelSpeed());
        joystick.b().onTrue(ShooterSubsystem.increaseLowerWheelSpeed());

        // Original rightTrigger binding — uncomment to restore:
        joystick.rightTrigger().whileTrue(
            Commands.parallel(
                Commands.startEnd(
                    () -> ThroatAndIndexerSubsystem.runMotor(),
                    () -> { ThroatAndIndexerSubsystem.stopMotorThroat(); ThroatAndIndexerSubsystem.stopMotorIndexer(); },
                    ThroatAndIndexerSubsystem
                ),
                Commands.startEnd(
                    () -> { ShooterSubsystem.runMotor1(1); ShooterSubsystem.runMotor2(-1); ShooterSubsystem.runBackspinMotor(1); }, // positive, negative, positive
                    () -> ShooterSubsystem.stopMotor(),
                    ShooterSubsystem
                ),
                Commands.startEnd(
                    () -> { intaketestSubsystem.IntakeOcilateCommand(); },
                    () -> intaketestSubsystem.IntakeUpCommand(),
                    intaketestSubsystem),
                Commands.repeatingSequence(
                    Commands.waitSeconds(1),
                    ThroatAndIndexerSubsystem.reverseMotorCommand()
                )
            )
        );

//////////////////////////////////////////////////////////////////////////////
/// COPILOT CONTROLS
//////////////////////////////////////////////////////////////////////////////

        copilotController.x().onTrue(
            Commands.sequence(
                intaketestSubsystem.IntakeDownCommand(),
                Commands.waitSeconds(0.5),
                intakeWheelSubsystem.runMotorCommand()
            )
        );
        
        copilotController.y().onTrue(
                Commands.sequence(intakeWheelSubsystem.stopMotorCommand(),
                intaketestSubsystem.intakeHalfWayCommand()
                )
        );  

        copilotController.b().onTrue(
                Commands.sequence(intakeWheelSubsystem.stopMotorCommand(),
                intaketestSubsystem.IntakeUpCommand()
                )
        ); 

       copilotController.a().whileTrue(
            Commands.startEnd(
                () -> intakeWheelSubsystem.runMotor(-0.2),
                () -> intakeWheelSubsystem.runMotorCommand(),
                intakeWheelSubsystem
            )
        ); 
        copilotController.leftBumper().whileTrue(turretSubsystem.aimAtHubViaVision());

        copilotController.rightBumper().onTrue(turretSubsystem.TurretToZero());
        

        // SmartDashboard.putData(autochooser);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Periodically called by the matchTime Notifier to update SmartDashboard.
     */
    private void updateMatchTimeOnDashboard() {
        double matchSeconds = DriverStation.getMatchTime();
        // DriverStation may return negative if match info not available
        if (Double.isNaN(matchSeconds) || matchSeconds < 0) {
            SmartDashboard.putString("Match Time", "--:--");
            SmartDashboard.putNumber("Match Time (s)", -1);
            return;
        }

        // Round up so display shows remaining whole seconds intuitively
        int secondsLeft = (int) Math.ceil(matchSeconds);
        int mins = secondsLeft / 60;
        int secs = secondsLeft % 60;
        String formatted = String.format("%02d:%02d", mins, secs);

        SmartDashboard.putString("Match Time", formatted);
        SmartDashboard.putNumber("Match Time (s)", matchSeconds);
    }

}