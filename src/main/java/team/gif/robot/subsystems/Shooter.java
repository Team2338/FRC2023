package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

/**
 * Controls all aspects of the arm
 */
/*
 * 0 position is approx compass heading of 350
 *    (offsets are used in constants to take into account straight up being non-zero)
 * 0 degrees is considered straight up
 */
public class Shooter extends SubsystemBase {

    public static WPI_TalonSRX leftMotor;
    public static WPI_TalonSRX rightMotor;

    public Shooter() {
        leftMotor = new WPI_TalonSRX(RobotMap.SHOOTER_LEFT_MOTOR);
        rightMotor = new WPI_TalonSRX(RobotMap.SHOOTER_RIGHT_MOTOR);
    }

    /**
     * Move the arm with an input current
     * @param percent percent of max current to move the arm
     */
    public void shoot(double percent) {
        leftMotor.set(-percent);
        rightMotor.set(percent);
    }
}
