package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLevelSubsystem extends SubsystemBase{
    private StatusSignal<Angle> motorPosition;
    
    private final double gearBoxRatio = 27;
    private final TalonFX intakeMotor;

   public StatusSignal<Angle> position;

    public IntakeLevelSubsystem() {
        intakeMotor = new TalonFX(14); 
        position = intakeMotor.getPosition();

TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.Slot0.kP = 4.8; // P value: Position
    cfg.Slot0.kI = 0; // I value: Integral
    cfg.Slot0.kD = 0.1; // D value: Derivative
    cfg.Slot0.kV = 0.12; // V value: Velocity
    //cfg.Slot0.kG = 0.1; // G value: Feedforward
    cfg.Slot0.kA = 0.01; // A value: Acceleration
    cfg.Slot0.kS = 0.25; // S value: Soft Limit

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 16; // Target cruise velocity of 80 rps
    mm.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    mm.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    intakeMotor.getConfigurator().apply(cfg);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(position);
       // System.out.println(position.getValueAsDouble() + " Intake Level motor");
    }

    public Command IntakeUpCommand() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        return Commands.sequence(
          Commands.runOnce(() -> {
              intakeMotor.setNeutralMode(NeutralModeValue.Brake);
              intakeMotor.setControl(m_request.withPosition(0));
          })
        );
    }

    public Command IntakeDownCommand() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        return Commands.sequence(
          Commands.runOnce(() -> {
              intakeMotor.setNeutralMode(NeutralModeValue.Coast);
              intakeMotor.setControl(m_request.withPosition(-10.2));
          })
        );
    }
}
