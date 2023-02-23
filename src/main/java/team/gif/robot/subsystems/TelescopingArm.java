package team.gif.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class TelescopingArm extends SubsystemBase {
    public static CANSparkMax telescopingMotor;

    public TelescopingArm() {
        telescopingMotor = new CANSparkMax(RobotMap.TELESCOPING_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        telescopingMotor.restoreFactoryDefaults();
        telescopingMotor.setInverted(true);
        telescopingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setMotorSpeed(double percent) {
        telescopingMotor.set(percent);
    }

    public double getPosition() {
        return telescopingMotor.getEncoder().getPosition();
    }

}
