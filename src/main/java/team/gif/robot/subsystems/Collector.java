package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Collector extends SubsystemBase {
//    public static WPI_TalonSRX leftMotor = new WPI_TalonSRX(RobotMap.COLLECTOR_LEFT_MOTOR);
    public static WPI_TalonSRX rightMotor = new WPI_TalonSRX(RobotMap.COLLECTOR_MOTOR);

    public Collector() {
//        leftMotor.configFactoryDefault();
//        leftMotor.setInverted(false);
//        leftMotor.setNeutralMode(NeutralMode.Brake); //setting to brake mode
//        leftMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
//        leftMotor.setSensorPhase(true);

        rightMotor.configFactoryDefault();
        rightMotor.setInverted(false);
        rightMotor.setNeutralMode(NeutralMode.Brake); //setting to brake mode
        rightMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
        rightMotor.setSensorPhase(true);
    }

    public void setSpeedPercentCollector(double percent) {
//        leftMotor.set(ControlMode.PercentOutput, percent);
        rightMotor.set(ControlMode.PercentOutput, percent);
    }

//    public double getLeftTicks() {
//        return leftMotor.getSelectedSensorPosition();
//    }

//    public double getRightTicks() {
//        return rightMotor.getSelectedSensorPosition();
//    }
}
