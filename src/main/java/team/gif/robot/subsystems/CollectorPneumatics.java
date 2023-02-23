package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class CollectorPneumatics extends SubsystemBase {
    private static final DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_COLLECTOR_RIGHT_FORWARD, RobotMap.SOLENOID_COLLECTOR_RIGHT_REVERSE);

    public void pneumaticsIn() {
        leftSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void pneumaticsOut() {
        leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
