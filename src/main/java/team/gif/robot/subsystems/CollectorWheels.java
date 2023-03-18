package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class CollectorWheels extends SubsystemBase {
    private static final DoubleSolenoid solenoid = new DoubleSolenoid( (Robot.isCompBot ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM), RobotMap.SOLENOID_COLLECTOR_FORWARD, RobotMap.SOLENOID_COLLECTOR_REVERSE);
    private DoubleSolenoid.Value state = DoubleSolenoid.Value.kForward;

    public void wheelsIn() {
        state = DoubleSolenoid.Value.kForward;
        Robot.ledSubsystem.LEDWheelsIn();
        Robot.limelightHigh.setPipeline(0);
    }

    public void wheelsOut() {
        state = DoubleSolenoid.Value.kReverse;
        Robot.ledSubsystem.LEDWheelsOut();
        Robot.limelightHigh.setPipeline(1);
    }

    public void setWheelState() {
        solenoid.set(state);
    }

    public boolean getWheelState() {
        if (solenoid.get() == DoubleSolenoid.Value.kReverse) { // true when wheels are out
            return true;
        } else {
            return false;
        }
    }
}