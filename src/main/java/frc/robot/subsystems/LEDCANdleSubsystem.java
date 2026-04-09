package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDCANdleSubsystem extends SubsystemBase {
    
    private final CANdle CANdle = new CANdle(35);
    private final CANdleConfiguration config = new CANdleConfiguration();

    public LEDCANdleSubsystem() {

        this.config.statusLedOffWhenActive = false;
        this.config.disableWhenLOS = false;
        this.config.stripType = LEDStripType.RGB;
        this.config.brightnessScalar = 1;
        this.config.vBatOutputMode = VBatOutputMode.Modulated;

        this.CANdle.configAllSettings(this.config, 100);

    }
    // If CANdle is Red, the bot is not in range to shoot, so the bot needs to move to the desired location
    public Command LEDRed() {
        return Commands.runOnce(() -> {
            this.CANdle.setLEDs(255, 0, 0);
        });
    }
    // If CANdle is Green, bot is in range to both align and shoot
    public Command LEDGreen() {
         return Commands.runOnce(() -> {
            this.CANdle.setLEDs(0, 255, 0);
        });
    }
    // If CANdle is White
        public Command LEDWhite() {
         return Commands.runOnce(() -> {
            this.CANdle.setLEDs(255, 255, 255);
        });
    }
    // If CANdle is Off, the bot is either off or dead
    public Command LEDOff() {
        return Commands.runOnce(() -> {
            this.CANdle.setLEDs(0, 0, 0);
        });
    }
 }

