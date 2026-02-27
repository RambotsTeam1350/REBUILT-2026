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
    
        private final TalonFX motor = new TalonFX(15); 
        private StatusSignal<Angle> motorPosition;

    public ClimberSubsystem() {

            TalonFXConfiguration cfg = new TalonFXConfiguration();
            cfg.Slot0.kP = 4.8;
            cfg.Slot0.kI = 0;
            cfg.Slot0.kD = 0.1;
            cfg.Slot0.kV = .12;
            cfg.Slot0.kA = .01;
            cfg.Slot0.kS = .25;

            this.motorPosition = this.motor.getPosition();
            MotionMagicConfigs mm = cfg.MotionMagic;
            mm.MotionMagicCruiseVelocity = 6; 
            mm.MotionMagicAcceleration = 80;
            mm.MotionMagicJerk = 1600; 
            motor.getConfigurator().apply(cfg);

    }

    @Override
    public void periodic() {
         System.out.println(motorPosition.getValueAsDouble() + " Climber motor");
    }

        public Command ClimbUpCommand() {
            return Commands.sequence(
                    Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(4)))
                );
            } 

            public Command ClimbDownCommand() {
            return Commands.sequence(
                    Commands.runOnce(() -> motor.setControl(new MotionMagicVoltage(0)))
                );
            }

}
