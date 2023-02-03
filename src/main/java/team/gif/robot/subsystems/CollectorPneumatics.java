package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class CollectorPneumatics extends SubsystemBase {
    private static final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_COLLECTOR_FORWARD, RobotMap.SOLENOID_COLLECTOR_REVERSE);

    public void collectorIn() {
        doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void collectorOut() {
        doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void collectorNeutral() {
        doubleSolenoid.set(DoubleSolenoid.Value.kOff);
    }

}
