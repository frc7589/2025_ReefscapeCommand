package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralSubsystem.IntakeState;

public class LEDSubsystem extends SubsystemBase{
    private CoralSubsystem m_Coral;

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

    private LEDPattern pattern = LEDPattern.kOff;

    private Timer m_Timer = new Timer();

    private int aStage = 0;
    private int CoralStage = 0;

    private boolean rainbow = false;

    private IntakeState intakeState = IntakeState.kEmpty;

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
        if (isBlink) pattern = pattern.blink(Seconds.of(0.075), Seconds.of(0.075));
    }

    public void setAStage (int stage) {
        aStage = stage;
    }

    public void setCoralStage (int stage) {
        CoralStage = stage;
    }

    public void setIntakeState (IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public void setRainbow (boolean rainbow) {
        this.rainbow = rainbow;
    }


    @Override
    public void periodic() {
        SmartDashboard.putString("LED",intakeState.name());
        SmartDashboard.putNumber("CoralStage", CoralStage);
        if (rainbow) setLEDColor(LEDColor.kRainbow, false);
        else{
            if (aStage == 1) {
                m_Timer.reset();
                m_Timer.start();
                aStage = 2;
            } else if (aStage == 2 && m_Timer.get() < 1.5) {
                setLEDColor(LEDColor.kPurple, true);
            } else if (intakeState == IntakeState.kLoading) {
                switch (CoralStage) {
                    case 0:
                        setLEDColor(LEDColor.kOrange, true);  
                        break;
                    case 1:
                        setLEDColor(LEDColor.kGreen, true);  
                        break;
                    case 2:
                        setLEDColor(LEDColor.kGreen, true);
                        break;
                    case 3:
                        setLEDColor(LEDColor.kGreen, true);
                        break;
                    case 4:
                        setLEDColor(LEDColor.kGreen, false);
                        break;
                    default:
                        break;
                }
            } else if (intakeState == IntakeState.kLoad) {
                setLEDColor(LEDColor.kGreen, false);
            } else {
                setLEDColor(LEDColor.kRed, false);
            }
        }
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }
}
