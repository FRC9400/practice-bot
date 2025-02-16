package frc.robot.Subsystems.LEDs;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canIDConstants;

public class LEDs extends SubsystemBase {
    /* CANdle Objects */
    private CANdle CANdle;
    private Color color;
    private CANdleConfiguration CANdleConfigs;
    private Color8Bit color8bit;

    public LEDs(){
        /* CANdle Objects */
        CANdle = new CANdle(canIDConstants.CANdle, canIDConstants.canivore);
        color = new Color();
        CANdleConfigs = new CANdleConfiguration();
        color8bit = new Color8Bit(color);


        /* CANdle Configuration */
        CANdleConfigs.stripType = LEDStripType.GRB;
        CANdleConfigs.brightnessScalar = 0.5;
        CANdle.configAllSettings(CANdleConfigs);
    }

    public void setColor(){
        CANdle.setLEDs(0,0,255);
    }
}
