// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake.IntakeLevelSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Pigeon2 pigeon = new Pigeon2(0, "rio");
  private final RobotContainer m_robotContainer;
  private final TurretSubsystem turretSubsystem;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final IntakeLevelSubsystem intakeLevelSubsystem = new IntakeLevelSubsystem();
 

  public Robot() {
    m_robotContainer = new RobotContainer();

    turretSubsystem = new TurretSubsystem(drivetrain.getPoseEstimator());
    //CommandScheduler.registerSubsystem(turretSubsystem);
     
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    BaseStatusSignal.refreshAll(intakeLevelSubsystem.position);
    
  } 

  @Override
  public void disabledInit() {
    double currentHeading = drivetrain.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees();
     LimelightHelpers.SetRobotOrientation("limelight-fifteen", currentHeading, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-three", currentHeading, 0, 0, 0, 0, 0);
  
      LimelightHelpers.SetIMUMode("limelight-three", 1);
  LimelightHelpers.SetIMUMode("limelight-fifteen", 1);
  }

  @Override
  public void disabledPeriodic() {
  }
  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    LimelightHelpers.SetIMUMode("limelight-three", 3);
    LimelightHelpers.SetIMUMode("limelight-fifteen", 3);
  }

  @Override
  public void teleopPeriodic() {

}
  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public void robotInit() {
  CameraServer.startAutomaticCapture();
  pigeon.setYaw(0);
  }
}



