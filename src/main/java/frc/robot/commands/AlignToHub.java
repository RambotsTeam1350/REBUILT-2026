package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class AlignToHub extends Command 
    {  
        private CommandSwerveDrivetrain drivetrain;
        private LimelightTarget_Detector limelight;

                private static final PIDController xpid = new PIDController(0.55, 0, 0.5, 0.1);
                private static final PIDController ypid = new PIDController(0.55, 0, 0.5, 0.1);
                private static final PIDController rotatorpid = new PIDController(0.55, 0, 0.5, 0.1);
        
                private static int offsetInches;
        
        public void alignToHub(CommandSwerveDrivetrain drivetrain, LimelightTarget_Detector limelight, int offsetInches) 
         {
            this.drivetrain = drivetrain;
            this.limelight = limelight; 
            AlignToHub.offsetInches = offsetInches;

        }

        @Override
        public void initialize() 
        {

        }

        @Override
        public void execute()
        {
            LimelightTarget_Fiducial fiducial = null;
        }
    }
