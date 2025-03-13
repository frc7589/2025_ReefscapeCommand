package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private AddressableLED m_led = new AddressableLED(9);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(148);

    private static final Distance kLedSpacing = Meters.of(1 / 148.0);

    private LEDPattern white = LEDPattern.solid(Color.kWhite);
    private LEDPattern red = LEDPattern.solid(Color.kRed);
    private LEDPattern bule = LEDPattern.solid(Color.kBlue);
    private LEDPattern green = LEDPattern.solid(Color.kGreen);
    private LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    private LEDPattern orange = LEDPattern.solid(Color.kOrange);
    private LEDPattern purple = LEDPattern.solid(Color.kPurple);
    private LEDPattern pink = LEDPattern.solid(Color.kPink);
    private LEDPattern off = LEDPattern.kOff;

    
    //private final LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue, Color.kRed);

    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    /*
    private AddressableLEDBufferView m_A = m_ledBuffer.createView(0, 50);
    private AddressableLEDBufferView m_B = m_ledBuffer.createView(50, 100);
    */

    public static enum LEDColor{
        kWhite,
        kRed,
        kBlue,
        kGreen,
        kYellow,
        kOrange,
        kPurple,
        kPink,
        kRainbow,
        kOff
    }

    public LEDSubsystem () {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setLEDColor(LEDColor color, boolean isBlink) {
        LEDPattern pattern = LEDPattern.kOff;
        switch (color) {
            case kWhite:
                pattern = white;
                break;
            case kRed:
                pattern = red;
                break;
            case kBlue:
                pattern = bule;
                break;
            case kGreen:
                pattern = green;
                break;
            case kYellow:
                pattern = yellow;
                break;
            case kOrange:
                pattern = orange;
                break;
            case kPurple:
                pattern = purple;
                break;
            case kPink:
                pattern = pink;
                break;
            case kRainbow:
                pattern = m_scrollingRainbow;
                break;
            case kOff:
                pattern = off;
                break;
            default:
                pattern = white;
                break;
        }
        if (isBlink) pattern = pattern.blink(Seconds.of(0.3), Seconds.of(0.1));
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }
}
