package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;
import team.gif.robot.Constants;


public class LEDsubsystem extends SubsystemBase {
    private static AddressableLED led;
    private static AddressableLEDBuffer ledBuffer;

    public LEDsubsystem() {
        super();
        //initializes by inputting PWM port number from Roborio
        led = new AddressableLED(RobotMap.LED_PWM_PORT);

        //initialize by inputting length of LED (# of LED)
        ledBuffer = new AddressableLEDBuffer(Constants.LEDsubsystem.NUM_LEDS);
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
    }

    /**
     * Sets the specified LED to the RGB values
     * @param r red
     * @param g green
     * @param b blue
     */
    public void setLEDColor (int r, int g, int b) {
        for (var i = 0; i < Constants.LEDsubsystem.NUM_LEDS; i++) {
            ledBuffer.setRGB(i, r, g, b);
            led.setData(ledBuffer);
        }
    }
}

