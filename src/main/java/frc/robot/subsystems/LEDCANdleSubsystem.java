package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDCANdleSubsystem extends SubsystemBase {

    private final CANdle candle = new CANdle(35);

    private final SolidColor redControl = new SolidColor(0, 511).withColor(new RGBWColor(255, 0, 0));
    private final SolidColor greenControl = new SolidColor(0, 511).withColor(new RGBWColor(0, 255, 0));
    private final SolidColor whiteControl = new SolidColor(0, 511).withColor(new RGBWColor(255, 255, 255));
    private final SolidColor offControl = new SolidColor(0, 511).withColor(new RGBWColor(0, 0, 0));

    public LEDCANdleSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();

        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = 1.0;
        config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;

        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
        config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;

        candle.getConfigurator().apply(config);
    }

    // If CANdle is Red, the bot is not in range to shoot, so the bot needs to move
    // to the desired location
    public Command LEDRed() {
        return Commands.runOnce(() -> candle.setControl(redControl));
    }

    // If CANdle is Green, bot is in range to both align and shoot
    public Command LEDGreen() {
        return Commands.runOnce(() -> candle.setControl(greenControl));
    }

    // If CANdle is White
    public Command LEDWhite() {
        return Commands.runOnce(() -> candle.setControl(whiteControl));
    }

    // If CANdle is Off, the bot is either off or dead
    public Command LEDOff() {
        return Commands.runOnce(() -> candle.setControl(offControl));
    }
}
