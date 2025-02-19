package frc.robot.Subsystems.LEDs;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.canIDConstants;

public class LEDs {
    /* CANdle Objects */
    private CANdle CANdle;
    private Color color;
    private LEDStates LEDstate;
    private BlinkPattern blinkPattern;
    private Timer blinkTimer;
    private CANdleConfiguration CANdleConfigs;
    private Color8Bit color8bit;
    private double onDuration;
    private double offDuration;

    public enum LEDStates{
        DISABLED,
        IDLE,
        INTAKING,
        INTOOK,
        SCORING,
        DEALGAING,
        SCORED,
        PROCESSING,
        PROCESSED,
        HEIGHTSET
    }

    public enum BlinkPattern{
        SOLID,
        BLINK_SLOW,
        BLINK_FAST
    }

    public LEDs(){
        /* CANdle Objects */
        CANdle = new CANdle(canIDConstants.CANdle, canIDConstants.canivore);
        color = new Color();
        LEDstate = LEDStates.DISABLED;
        blinkPattern = BlinkPattern.SOLID;
        blinkTimer = new Timer();
        CANdleConfigs = new CANdleConfiguration();

        /* CANdle Configuration */
        CANdleConfigs.stripType = LEDStripType.GRB;
        CANdleConfigs.brightnessScalar = 1;
        CANdle.configAllSettings(CANdleConfigs);

        onDuration = 0;
        offDuration = 0;
    }
    

    public void Loop(){
        Logger.recordOutput("LEDState", LEDstate);
        switch(LEDstate){
            case DISABLED:
                color = Color.kRed;
                blinkPattern = BlinkPattern.SOLID;
                break;
            case IDLE:
                color = Color.kBeige;
                blinkPattern = BlinkPattern.BLINK_SLOW;
                break;
            case INTAKING:
                color = Color.kBlue;
                blinkPattern = BlinkPattern.BLINK_FAST;
                break;
            case INTOOK:
                color = Color.kBlue;
                blinkPattern = BlinkPattern.SOLID;
                break;
            case SCORING:
                color = Color.kDarkCyan;
                blinkPattern = BlinkPattern.BLINK_FAST;
                break;
            case DEALGAING:
                color = Color.kCyan;
                blinkPattern = BlinkPattern.BLINK_FAST;
                break;
            case SCORED:
                color = Color.kCrimson;
                blinkPattern = BlinkPattern.SOLID;
                break;
            case PROCESSING:
                color = Color.kYellowGreen;
                blinkPattern = BlinkPattern.BLINK_FAST;
                break;
            case PROCESSED:
                color = Color.kGreen;
                blinkPattern = BlinkPattern.SOLID;
            case HEIGHTSET:
                color = Color.kPaleTurquoise;
                blinkPattern = BlinkPattern.BLINK_SLOW;
                break;
        }

        color8bit = new Color8Bit(color);
        applyBlinkPattern();
    }

    public void applyBlinkPattern() {
        if (blinkPattern == BlinkPattern.SOLID) {
            CANdle.setLEDs(color8bit.red, color8bit.green, color8bit.blue);
            return;
        } 
        double time = blinkTimer.get();
    
        if (blinkPattern == BlinkPattern.BLINK_FAST) {
            onDuration = 0.08;
            offDuration = 0.16;
        } else if (blinkPattern == BlinkPattern.BLINK_SLOW) {
            onDuration = 0.25;
            offDuration = 0.50;
        }
    
        if (time >= onDuration + offDuration) {
            blinkTimer.reset();
        } else if (time >= onDuration) {
            CANdle.setLEDs(0, 0, 0);
        } else {
            CANdle.setLEDs(color8bit.red, color8bit.green, color8bit.blue);
        }
    }

    public void setState(LEDStates nextState){
        LEDstate = nextState;
    }

    public void requestDisabledLED() {
        setState(LEDStates.DISABLED);
    }

    public void requestIdleLED() {
        setState(LEDStates.IDLE);
    }

    public void requestIntakingLED() {
        setState(LEDStates.INTAKING);
    }

    public void requestIntookLED() {
        setState(LEDStates.INTOOK);
    }

    public void requestScoringLED() {
        setState(LEDStates.SCORING);
    }

    public void requestDealgaingLED() {
        setState(LEDStates.DEALGAING);
    }

    public void requestScoredLED() {
        setState(LEDStates.SCORED);
    }

    public void requestProcessingLED() {
        setState(LEDStates.PROCESSING);
    }

    public void requestProcessedLED() {
        setState(LEDStates.PROCESSED);
    }

    public void requestHeightSetLED() {
        setState(LEDStates.HEIGHTSET);
    }

}
