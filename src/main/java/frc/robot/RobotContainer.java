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
	// private final ShooterAimSubsystem shooterAimSubsystem;
	private final SendableChooser<Command> autoChooser;
	private LimelightTarget_Detector limelight = new LimelightTarget_Detector();
	// Notifier to update the dashboard match-time widget periodically
	private final Notifier matchTimeNotifier;

	// private final SparkFlex motor = new SparkFlex(16, MotorType.kBrushless);

	public RobotContainer() {
		// Explicitly ensure the drivetrain configures AutoBuilder before attempting
		// to build the chooser. If configuration fails, fall back to an empty chooser
		// to avoid crashing the robot code.

		// Construct the turret after the drivetrain so we can pass the drivetrain's
		// pose estimator into the turret constructor.
		turretSubsystem = new TurretSubsystem(drivetrain.getPoseEstimator());
		// ShooterAimSubsystem uses TurretSubsystem as its distance source so both
		// subsystems share the same turret-corrected distance to hub.
		// shooterAimSubsystem = new ShooterAimSubsystem(turretSubsystem);

		configureBindings();

		//////////////////////////////////////////
		/// Path planner autos
		///

		NamedCommands.registerCommand("runIntakeMotor", intakeWheelSubsystem.runMotorCommand());
		NamedCommands.registerCommand("stopIntakeMotor", intakeWheelSubsystem.stopMotorCommand());
		NamedCommands.registerCommand("IntakeDownCommand", intaketestSubsystem.IntakeDownCommand());
		NamedCommands.registerCommand("IntakeUpCommand", intaketestSubsystem.IntakeUpCommand());
		NamedCommands.registerCommand("IntakeHalfUpCommand", intaketestSubsystem.intakeHalfWayCommand());
		NamedCommands.registerCommand("AimTurret", turretSubsystem.setTurretPositionVariable());
		NamedCommands.registerCommand(
			"RevShooterMotor",
			Commands.run(
				() -> ShooterSubsystem.standbyMotor()
			));
		NamedCommands.registerCommand(
				"stopMotorCommand",
				Commands.parallel(
						ThroatAndIndexerSubsystem.stopMotorCommand(),
						ShooterSubsystem.stopMotorCommand()));
		NamedCommands.registerCommand(
				"runMotorCommand",
				Commands.parallel(
						ThroatAndIndexerSubsystem.runMotorCommand(),
						Commands.run(
							() -> ShooterSubsystem.runShooterWithAutoVelocity(turretSubsystem.getDistanceToHub()),
							turretSubsystem
							)
						) 
					);

		///////////////////////////////////////////////////////////////

		// Start a background notifier to update the match time on the dashboard
		matchTimeNotifier = new Notifier(this::updateMatchTimeOnDashboard);
		// update twice per second
		matchTimeNotifier.startPeriodic(0.5);

		NamedCommands.registerCommand("TurretAutoAimToHub", turretSubsystem.TurretAutoAimToHub());
		NamedCommands.registerCommand("ClimbDownCommand", climberSubsystem.ClimbDownCommand());

		autoChooser = AutoBuilder.buildAutoChooser("middle boring");
		SmartDashboard.putData("Auto Chooser", autoChooser);
		SmartDashboard.putNumber("lower motor speed", ShooterSubsystem.shooterTargetRPM);
		SmartDashboard.putNumber("backspin motor speed", ShooterSubsystem.backspinTargetRPM);
	}

private double[] getHubTarget() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new double[] { turretSubsystem.HUB_RED_X, turretSubsystem.HUB_Y };
        }
        return new double[] { turretSubsystem.HUB_BLUE_X, turretSubsystem.HUB_Y };
    }

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically.
				// Squared input curve: preserves sign but squares magnitude for finer low-speed
				// control.
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

		// joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
		/*
		 * joystick.b().whileTrue(drivetrain.applyRequest(
		 * () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
		 * -joystick.getLeftX()))));
		 */
		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		//////////////////////////////////////////////////////////////////////////////
		/// DRIVER CONTROLS
		//////////////////////////////////////////////////////////////////////////////

		// reset the field-centric heading on left bumper press
		joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		// choose between pose aim test or center the turret
	/* 	copilotController.rightBumper()
				.onTrue(turretSubsystem.setTurretPosition(turretSubsystem.turretDegreesAndEncoderUnits(0)));
*/
	/* 
		joystick.x().onTrue(ShooterSubsystem.decreaseBackspinWheelSpeed());
		joystick.y().onTrue(ShooterSubsystem.increaseBackspinWheelSpeed());
		joystick.a().onTrue(ShooterSubsystem.decreaseLowerWheelSpeed());
		joystick.b().onTrue(ShooterSubsystem.increaseLowerWheelSpeed());
	*/

		// D-pad up: auto-velocity hub shot — aim first, then shoot after 0.25 s.
		// RPMs are calculated from live distance each loop cycle once shooting starts.
		// Use this once the interpolation table is calibrated.
		joystick.povUp().whileTrue(
				Commands.sequence(
						turretSubsystem.setTurretPositionVariable(),
						Commands.waitSeconds(0.25),
						Commands.run(
								() -> ShooterSubsystem.runShooterWithAutoVelocity(turretSubsystem.getDistanceToHub()),
								ShooterSubsystem)
				).finallyDo(interrupted -> ShooterSubsystem.standbyMotor())); //finallyDo interupted is a substitute for startEnd

		// Left trigger: lob shot — aims turret toward alliance zone and fires.
		// Use when collecting in mid-field and the hub is not lit.
		/*
		joystick.leftTrigger().whileTrue(
				Commands.parallel(
						turretSubsystem.aimForLobShot(),
						Commands.startEnd(
								() -> ThroatAndIndexerSubsystem.runMotor(),
								() -> {
									ThroatAndIndexerSubsystem.stopMotorThroat();
									ThroatAndIndexerSubsystem.stopMotorIndexer();
								},
								ThroatAndIndexerSubsystem),
						Commands.startEnd(
								ShooterSubsystem::runShooter,
								ShooterSubsystem::stopMotor,
								ShooterSubsystem)));
 */

 joystick.leftTrigger().whileTrue(
				Commands.parallel(
						//turretSubsystem.setTurretPositionVariable(), // replace with zero positioning if turret aiming fails
						Commands.startEnd(
								() -> ThroatAndIndexerSubsystem.runMotor(),
								() -> {
									ThroatAndIndexerSubsystem.stopMotorThroat();
									ThroatAndIndexerSubsystem.stopMotorIndexer();
								},
								ThroatAndIndexerSubsystem),
						Commands.startEnd(
								ShooterSubsystem::runShooter, //runs at a set speed, josh will drive to aim, backup if data-table fails
								ShooterSubsystem::standbyMotor,
								ShooterSubsystem),
						Commands.startEnd(
								() -> {
									intaketestSubsystem.IntakeOcilateCommand();
								},
								() -> intaketestSubsystem.IntakeUpCommand(),
								intaketestSubsystem),
						Commands.repeatingSequence(
								Commands.waitSeconds(1),
								ThroatAndIndexerSubsystem.reverseMotorCommand())));

		// Right trigger: hub shot — aims turret at hub and fires.
		// On release, flywheels drop to standby RPM rather than stopping so the
		// heavy flywheels stay in motion and reach full speed faster on the next shot.
		joystick.rightTrigger().whileTrue(
				Commands.parallel(
						//turretSubsystem.setTurretPositionVariable(), // replace with zero positioning if turret aiming fails
						Commands.startEnd(
								() -> ThroatAndIndexerSubsystem.runMotor(),
								() -> {
									ThroatAndIndexerSubsystem.stopMotorThroat();
									ThroatAndIndexerSubsystem.stopMotorIndexer();
								},
								ThroatAndIndexerSubsystem),
						Commands.startEnd(
								() -> ShooterSubsystem.runShooterWithAutoVelocity(turretSubsystem.getDistanceToHub()), //runs at a set speed, josh will drive to aim, backup if data-table fails
								ShooterSubsystem::standbyMotor,
								ShooterSubsystem),
						Commands.startEnd(
								() -> {
									intaketestSubsystem.IntakeOcilateCommand();
								},
								() -> intaketestSubsystem.IntakeUpCommand(),
								intaketestSubsystem),
						Commands.repeatingSequence(
								Commands.waitSeconds(1),
								ThroatAndIndexerSubsystem.reverseMotorCommand())));

		//////////////////////////////////////////////////////////////////////////////
		/// COPILOT CONTROLS
		//////////////////////////////////////////////////////////////////////////////

		copilotController.x().onTrue(
				Commands.sequence(
						intaketestSubsystem.IntakeDownCommand(),
						Commands.waitSeconds(0.5),
						intakeWheelSubsystem.runMotorCommand()));

		copilotController.y().onTrue(
				Commands.sequence(intakeWheelSubsystem.stopMotorCommand(),
						intaketestSubsystem.intakeHalfWayCommand()));

		copilotController.b().onTrue(
				Commands.sequence(intakeWheelSubsystem.stopMotorCommand(),
						intaketestSubsystem.IntakeUpCommand()));

		copilotController.a().whileTrue(
				Commands.startEnd(
						() -> intakeWheelSubsystem.runMotor(-0.2),
						() -> intakeWheelSubsystem.runMotorCommand(),
						intakeWheelSubsystem));
		// copilotController.leftBumper().whileTrue(turretSubsystem.setTurretPosition(3));

		/* copilotController.rightBumper().onTrue(Commands.parallel(
				turretSubsystem.setTurretPositionVariable(),
				Commands.runOnce(() -> turretSubsystem.updateTurretAngle())
			)
		);*/
		copilotController.rightBumper().onTrue(turretSubsystem.setTurretPositionVariable()); //for shooting
		copilotController.leftBumper().onTrue(turretSubsystem.setTurretPosition(turretSubsystem.turretDegreesAndEncoderUnits(0))); //for feeding
		/*copilotController.rightTrigger()
				.onTrue(turretSubsystem.setTurretPosition(turretSubsystem.turretDegreesAndEncoderUnits(0)));
*/ /* 
	copilotController.rightTrigger().whileTrue(
				Commands.parallel(
						Commands.repeatingSequence(
								turretSubsystem.setTurretPositionVariable(),
								Commands.waitSeconds(0.5)), // re-aims every 0.5s while trigger held
						Commands.startEnd(
								() -> ThroatAndIndexerSubsystem.runMotor(),
								() -> {
									ThroatAndIndexerSubsystem.stopMotorThroat();
									ThroatAndIndexerSubsystem.stopMotorIndexer();
								},
								ThroatAndIndexerSubsystem),
						Commands.startEnd(
								() -> ShooterSubsystem.runShooterWithAutoVelocity(turretSubsystem.getDistanceToHub()), //change this, should be on copilot controller
								ShooterSubsystem::standbyMotor,
								ShooterSubsystem),
						Commands.startEnd(
								() -> {
									intaketestSubsystem.IntakeOcilateCommand();
								},
								() -> intaketestSubsystem.IntakeUpCommand(),
								intaketestSubsystem),
						Commands.repeatingSequence(
								Commands.waitSeconds(1),
								ThroatAndIndexerSubsystem.reverseMotorCommand())));

				// ^ For Shooting at the hub
			*/
copilotController.leftTrigger().whileTrue(
				Commands.parallel(
						//turretSubsystem.setTurretPositionVariable(), // replace with zero positioning if turret aiming fails
						Commands.startEnd(
								() -> ThroatAndIndexerSubsystem.runMotor(),
								() -> {
									ThroatAndIndexerSubsystem.stopMotorThroat();
									ThroatAndIndexerSubsystem.stopMotorIndexer();
								},
								ThroatAndIndexerSubsystem),
						Commands.startEnd(
								ShooterSubsystem::runShooter, //feeding, so auto-speed not required
								ShooterSubsystem::standbyMotor,
								ShooterSubsystem),
						Commands.startEnd(
								() -> {
									intaketestSubsystem.IntakeOcilateCommand();
								},
								() -> intaketestSubsystem.IntakeUpCommand(),
								intaketestSubsystem),
						Commands.repeatingSequence(
								Commands.waitSeconds(1),
								ThroatAndIndexerSubsystem.reverseMotorCommand())));

				//	^ For feeding

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
