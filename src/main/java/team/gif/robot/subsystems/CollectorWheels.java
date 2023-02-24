package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class CollectorWheels extends SubsystemBase {
    private static final DoubleSolenoid solenoid = new DoubleSolenoid( (Robot.isCompBot ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM), RobotMap.SOLENOID_COLLECTOR_FORWARD, RobotMap.SOLENOID_COLLECTOR_REVERSE);

    public void wheelsIn() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void wheelsOut() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }
}