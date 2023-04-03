package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class CollectorWheels extends SubsystemBase {
    private static final DoubleSolenoid solenoid = new DoubleSolenoid( (Robot.isCompBot ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM), RobotMap.SOLENOID_COLLECTOR_FORWARD, RobotMap.SOLENOID_COLLECTOR_REVERSE);
    private DoubleSolenoid.Value state = DoubleSolenoid.Value.kForward;

    /**
     * Sets the state of the collector wheel position to the in position
     */
    public void wheelsIn() {
        state = DoubleSolenoid.Value.kForward;
        Robot.ledSubsystem.LEDWheelsIn();
       // Robot.limelightHigh.setPipeline(0);
    }

    /**
     * Sets the state of the collector wheel position to the out position
     */
    public void wheelsOut() {
        state = DoubleSolenoid.Value.kReverse;
        Robot.ledSubsystem.LEDWheelsOut();
        //Robot.limelightHigh.setPipeline(1);
    }

    /**
     * Sets the state of the collector wheel position
     */
    public void setWheelState() {
        solenoid.set(state);
    }

    /**
     * Gets the state of the collector wheel position
     *
     * @return boolean - true if wheels are out (cube), false if wheels are in (cone)
     */
    public boolean getWheelState() {
        if (solenoid.get() == DoubleSolenoid.Value.kReverse) {
            return true;
        } else {
            return false;
        }
    }
}