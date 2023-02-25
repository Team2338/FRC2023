package team.gif.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class TelescopingArm extends SubsystemBase {
    public static CANSparkMax telescopingMotor;

    public TelescopingArm() {
        telescopingMotor = new CANSparkMax(RobotMap.TELESCOPING_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        telescopingMotor.restoreFactoryDefaults();
        telescopingMotor.setInverted(true);
        telescopingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        // would use soft limits but only takes in integer values which is not precise enough
    }

    public void setMotorSpeed(double percent) {
        // do not allow for arm to go past soft limits
        if ( (percent > 0 && telescopingMotor.getEncoder().getPosition() > Constants.TelescopingArm.MAX_POS ) ||
             (percent < 0 && telescopingMotor.getEncoder().getPosition() < Constants.TelescopingArm.MIN_POS ) )
            percent = 0;

        telescopingMotor.set(percent);
    }

    public double getPosition() {
        return telescopingMotor.getEncoder().getPosition();
    }
}
