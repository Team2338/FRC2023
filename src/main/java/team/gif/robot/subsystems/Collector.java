package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Collector extends SubsystemBase {
    public static WPI_TalonSRX collectorMotor = new WPI_TalonSRX(RobotMap.COLLECTOR_MOTOR);

    public Collector() {
        collectorMotor.configFactoryDefault();
        collectorMotor.setInverted(false);
        collectorMotor.setNeutralMode(NeutralMode.Brake); //setting to brake mode
        collectorMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
        collectorMotor.setSensorPhase(true);
    }

    public void setSpeedPercentCollector(double percent) {
        collectorMotor.set(ControlMode.PercentOutput, percent);
    }
}
