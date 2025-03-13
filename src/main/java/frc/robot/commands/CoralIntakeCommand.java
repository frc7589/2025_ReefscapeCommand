package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDColor;


public class CoralIntakeCommand extends Command {
    private CoralSubsystem m_intake;
    private LEDSubsystem m_led;
    private Timer m_Isfinished = new Timer();
    private Timer m_reverseTimer = new Timer();
    private int stage = 0;

    public CoralIntakeCommand(CoralSubsystem m_intake, LEDSubsystem m_led){
        this.m_intake = m_intake;
        this.m_led = m_led;
        addRequirements(m_intake);
        addRequirements(m_led);
    }

    @Override
    public void initialize(){
        System.out.println("CoralIntakeCommand scucess");
        m_reverseTimer.reset();
        m_Isfinished.reset();
        m_Isfinished.start();
        stage = 0;
        m_led.setLEDColor(LEDColor.kOrange, true);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("stage", stage);
        switch(stage) {
            case 0:
                if (m_intake.hasCoral()) stage++;
                else m_intake.intake();
                break;
            case 1:
                m_led.setLEDColor(LEDColor.kGreen, true);
                if(m_intake.hasCoral()) m_intake.slowMotor();
                else stage++;
                break;
            case 2:
                if(!m_intake.hasCoral()) m_intake.reverseMotor();
                else {
                    m_reverseTimer.start();
                    stage++;
                }
                break;
            case 3:
                if(m_reverseTimer.get() >= 0.12) stage++;
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (stage == 4) m_led.setLEDColor(LEDColor.kGreen, false);
        else m_led.setLEDColor(LEDColor.kRed, true);
        m_reverseTimer.stop();
        m_intake.stop();
    }

    @Override
    public boolean isFinished(){
        return stage == 4 || m_Isfinished.get() > 5;
    }
}            
