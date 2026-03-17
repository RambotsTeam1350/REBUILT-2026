package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsystem extends SubsystemBase { 

    // change the moter ID for the climber when we know what it is
    
        private final TalonFX climbermotor1; 
        private final TalonFX climbermotor2;
        private StatusSignal<Angle> motorPosition1;
        private StatusSignal<Angle> motorPosition2;

    public ClimberSubsystem() {
        climbermotor1 = new TalonFX(17); 
        climbermotor2 = new TalonFX(19);
            this.motorPosition1 = this.climbermotor1.getPosition();
            this.motorPosition2 = this.climbermotor2.getPosition();

            TalonFXConfiguration cfg = new TalonFXConfiguration();
            cfg.Slot0.kP = 4.8;
            cfg.Slot0.kI = 0;
            cfg.Slot0.kD = 0.1;
            cfg.Slot0.kV = .12;
            cfg.Slot0.kA = .01;
            cfg.Slot0.kS = .25;

            
            MotionMagicConfigs mm = cfg.MotionMagic;
            mm.MotionMagicCruiseVelocity = 6; 
            mm.MotionMagicAcceleration = 80;
            mm.MotionMagicJerk = 1600; 
            climbermotor1.getConfigurator().apply(cfg);
            climbermotor2.getConfigurator().apply(cfg);

    }

    @Override
    public void periodic() {

        motorPosition1.getValueAsDouble();
        motorPosition2.getValueAsDouble();
        //System.out.println(motorPosition1.getValueAsDouble() + " Climber motor 1, " + motorPosition2.getValueAsDouble() + " Climber motor 2");
    }

        public Command ClimbUpCommand() {
            return Commands.sequence(
                    Commands.runOnce(() -> climbermotor1.setControl(new MotionMagicVoltage(26.8))),
                    Commands.runOnce(() -> climbermotor2.setControl(new MotionMagicVoltage(26.3)))
    
        
                );
            } 

        

            public Command ClimbDownCommand() {
            return Commands.sequence(
                    Commands.runOnce(() -> climbermotor1.setControl(new MotionMagicVoltage(0)))
                
                );
            }

}
