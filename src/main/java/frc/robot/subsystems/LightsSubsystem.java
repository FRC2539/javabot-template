package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(LightsConstants.CANDLE_PORT);

    public static final Color orange = new Color(255, 25, 0);

    private Animation desiredAnimation = null;
    private Color desiredColor = null;

    private final Animation defaultAnimation =
            new ColorFlowAnimation(255, 25, 0, 0, 1, LightsConstants.LED_COUNT, Direction.Forward);

    public LightsSubsystem() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 1.0;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(candleConfiguration, 100);
    }

    public void setColor(Color color) {
        desiredColor = color;
        desiredAnimation = null;
    }

    public void setColor(int r, int g, int b) {
        setColor(new Color(r, g, b));
    }

    public void setBrightness(double percent) {
        candle.configBrightnessScalar(percent, 100);
    }

    public void setFlowAnimation(Color color, double speed) {
        desiredAnimation = new ColorFlowAnimation(
                color.red, color.green, color.blue, 0, speed, LightsConstants.LED_COUNT, Direction.Forward);
    }

    public void setFadeAnimation(Color color, double speed) {
        desiredAnimation =
                new SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, LightsConstants.LED_COUNT);
    }

    public void setBandAnimation(Color color, double speed) {
        desiredAnimation = new LarsonAnimation(
                color.red, color.green, color.blue, 0, speed, LightsConstants.LED_COUNT, BounceMode.Front, 3);
    }

    public void setStrobeAnimation(Color color, double speed) {
        desiredAnimation = new StrobeAnimation(color.red, color.green, color.blue, 0, speed, LightsConstants.LED_COUNT);
    }

    public void setRainbowAnimation(double speed) {
        desiredAnimation = new RainbowAnimation(1, 0.5, LightsConstants.LED_COUNT);
    }

    public CommandBase resetCommand() {
        return runOnce(() -> {
            desiredAnimation = null;
            desiredColor = null;
        });
    }

    @Override
    public void periodic() {
        if (desiredAnimation != null) {
            candle.animate(desiredAnimation);
        } else if (desiredColor != null) {
            candle.clearAnimation(0);
            setCANdleColor(desiredColor);
        } else {
            candle.animate(defaultAnimation);
        }
    }

    private void setCANdleColor(Color color) {
        candle.setLEDs(color.red, color.green, color.blue, 255, 0, LightsConstants.LED_COUNT);
    }

    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }
}
