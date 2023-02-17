package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static team.gif.robot.RobotMap.LED_LENGTH;
import static team.gif.robot.RobotMap.PWM_PORT;

public class LEDsubsystem extends SubsystemBase {
    private static AddressableLED led;
    private static AddressableLEDBuffer ledBuffer;

    public LEDsubsystem() {
        super();
        //initializes by inputting PWM port number from Roborio
        led = new AddressableLED(PWM_PORT);

        //initialize by inputting length of LEDs (# of LEDS)
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
    }

    public void setLEDYellow() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for Orange
            ledBuffer.setRGB(i, 60, 60, 0);

            led.setData(ledBuffer);
        }
    }

    public void setLEDPurple() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values to Purple
            ledBuffer.setRGB(i, 64, 0, 128);

            led.setData(ledBuffer);
        }
    }

    public void setLEDDefault() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for black (off)
            ledBuffer.setRGB(i, 0, 0, 0);

            led.setData(ledBuffer);
        }
    }

}

