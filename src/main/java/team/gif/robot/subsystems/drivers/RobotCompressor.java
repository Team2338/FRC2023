package team.gif.robot.subsystems.drivers;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotCompressor extends Compressor {
    public static AnalogInput pressureSensor = null;
    public RobotCompressor(int port, PneumaticsModuleType moduleType, int sensorID) {
        super(port, moduleType);
        pressureSensor = new AnalogInput(sensorID);
    }

    public double getPressure() {
        if (pressureSensor == null)
            return -1;
        else
            return 250 * (pressureSensor.getVoltage() / 4.82) - 25;
    }
}
