package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Arm extends SubsystemBase {
    public static WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.ARM_MOTOR);
    private static WPI_TalonSRX armEncoderTalon = new WPI_TalonSRX(RobotMap.ARM_ENCODER);
    private static MotorController armControl;

    private static final int MAX_SUPPLY_CURRENT_AMPS = 20;
    private static final int MAX_STATOR_CURRENT_AMPS = 90;

    private static double armTargetPos;

    public boolean armManualFlag = false;

    public Arm() {
        //motor controller groups
//        armControl = new MotorControllerGroup(armMotor);

        currentLimitingEnable(true); //limits

        //armMotor settings
        armMotor.setNeutralMode(NeutralMode.Brake); //setting to brake mode
        armMotor.configFactoryDefault();
        armMotor.setInverted(true); //maybe we might change to true (IDK)
        armMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
        armMotor.enableCurrentLimit(false); //limiter
        armMotor.setSensorPhase(true);

        //armEncoderTalon settings
        armEncoderTalon.setNeutralMode(NeutralMode.Brake); //setting to brake mode
        armEncoderTalon.configFactoryDefault();
        armEncoderTalon.setInverted(true); //maybe we might change to true (IDK)
        armEncoderTalon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
        armEncoderTalon.enableCurrentLimit(false); //limiter
        armEncoderTalon.setSensorPhase(true);

        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        // PID
        armMotor.config_kF(0, Constants.Arm.FF); //feed forward
        armMotor.config_kP(0, Constants.Arm.P); //proportional
        armMotor.config_kI(0,Constants.Arm.I);
        armMotor.config_IntegralZone(0,200);

        // soft limits
        armMotor.configReverseSoftLimitEnable(true);
        armMotor.configReverseSoftLimitThreshold(Constants.Arm.TICKS_ABS_MIN);
        armMotor.configForwardSoftLimitEnable(true);
        armMotor.configForwardSoftLimitThreshold(Constants.Arm.TICKS_ABS_MAX);
    }

    // getting the ticks from the encoders.
    public double getPosition() {
        return armMotor.getSelectedSensorPosition(); // 4096
    }

    // getting the ticks from the encoders.
    public void move(double percent) {
        if( (percent > 0 && getPosition() < Constants.Arm.TICKS_ABS_MAX) ||
            (percent < 0 && getPosition() > Constants.Arm.TICKS_ABS_MIN)
        ) {
            armMotor.set(percent);
        }
        else
            armMotor.set(0);
    }

    public double getOutput(){
        return armMotor.getMotorOutputPercent();
    }

    public void PIDMove() {
        armMotor.set(ControlMode.Position, armTargetPos);
    }

    public double PIDError(){
        return armMotor.getClosedLoopError();
    }

    // limits
    public void currentLimitingEnable(boolean enableLimit) {
        armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(enableLimit, MAX_SUPPLY_CURRENT_AMPS,MAX_STATOR_CURRENT_AMPS, 0));
    }

    public void setArmTargetPos(double pos) {
        armTargetPos = pos;
    }

    public boolean isFinished() {
        return Math.abs(PIDError()) < Constants.Arm.PID_TOLERANCE;
    }

    public boolean getArmManualFlag() { return armManualFlag;}
}
