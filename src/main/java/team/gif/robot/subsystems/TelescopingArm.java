package team.gif.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class TelescopingArm extends SubsystemBase {
    public static CANSparkMax telescopingMotor;

    public static SparkMaxPIDController pidController;

    public TelescopingArm() {
        telescopingMotor = new CANSparkMax(RobotMap.TELESCOPING_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        telescopingMotor.restoreFactoryDefaults();
        telescopingMotor.setInverted(true);
        telescopingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pidController = telescopingMotor.getPIDController();
        pidController.setP(Constants.TelescopingArm.P);
        pidController.setFF(Constants.TelescopingArm.FF);
        pidController.setI(Constants.TelescopingArm.I);
        pidController.setD(Constants.TelescopingArm.D);
        // would use soft limits but only takes in integer values which is not precise enough
    }

    /**
     * Set the motor output of the telescoping arm as a percent of max speed
     * @param percent percent of max speed to run the telescoping arm
     */
    public void setMotorSpeed(double percent) {
        // do not allow for arm to go past soft limits
        if ( (percent > 0 && telescopingMotor.getEncoder().getPosition() > Constants.TelescopingArm.MAX_POS ) ||
             (percent < 0 && telescopingMotor.getEncoder().getPosition() < Constants.TelescopingArm.MIN_POS ) )
            percent = 0;

        telescopingMotor.set(percent);
    }

    /**
     * Get the current position of the telescoping arm
     * @return the current position of the telescoping arm
     */
    public double getPosition() {
        return telescopingMotor.getEncoder().getPosition();
    }

    public boolean safePos() { return getPosition() < 0.8 * Constants.TelescopingArm.MAX_POS; }
}
