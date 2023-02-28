package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class LEDSubsystem extends SubsystemBase {
    private static AddressableLED led;
    private static AddressableLEDBuffer ledBuffer;

    private final int [] HP = {0,0,0};
    private final int [] GamePiece = {0,0,0};

    public LEDSubsystem() {
        super();
        //initializes by inputting PWM port number from Roborio
        led = new AddressableLED(RobotMap.LED_PWM_PORT);

        //initialize by inputting length of LED (# of LED)
        ledBuffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS_TOTAL);
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
    public void setLEDHPColor(int r, int g, int b) {
        HP[0] = r;
        HP[1] = g;
        HP[2] = b;
    }

    public void setLEDGamePieceColor() {
        GamePiece[0] = 0;
        GamePiece[1] = 255;
        GamePiece[2] = 0;
    }

    public void clearLEDGamePieceColor() {
        GamePiece[0] = 0;
        GamePiece[1] = 0;
        GamePiece[2] = 0;
    }

    public void setColors() {
        for (var i = 0; i < RobotMap.HP_LEDS.length; i++) {
            ledBuffer.setRGB(RobotMap.HP_LEDS[i],HP[0],HP[1], HP[2]);
            led.setData(ledBuffer);
        }
        for (int j = 0; j < RobotMap.GAME_PIECE_LEDS.length; j++) {
            ledBuffer.setRGB(RobotMap.GAME_PIECE_LEDS[j], GamePiece[0], GamePiece[1], GamePiece[2]);
            led.setData(ledBuffer);
        }
    }
}