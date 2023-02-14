package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
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
//    private static WPI_TalonSRX armEncoderTalon = new WPI_TalonSRX(RobotMap.ARM_ENCODER);
    private static MotorController armControl;

    private static final int MAX_SUPPLY_CURRENT_AMPS = 20;
    private static final int MAX_STATOR_CURRENT_AMPS = 90;

    private static double armTargetPos;

    public boolean armManualFlag = false;

    public Arm() {
        //motor controller groups
//        armControl = new MotorControllerGroup(armMotor);

//        configArmTalon();
        currentLimitingEnable(false); //limits

        //armMotor settings
        armMotor.setNeutralMode(NeutralMode.Brake); //setting to brake mode
        armMotor.configFactoryDefault();
        armMotor.setInverted(true); //maybe we might change to true (IDK)
        armMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
        armMotor.enableCurrentLimit(false); //limiter
        armMotor.setSensorPhase(true);

        //armEncoderTalon settings
//        armEncoderTalon.setNeutralMode(NeutralMode.Brake); //setting to brake mode
//        armEncoderTalon.configFactoryDefault();
//        armEncoderTalon.setInverted(true); //maybe we might change to true (IDK)
//        armEncoderTalon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
//        armEncoderTalon.enableCurrentLimit(false); //limiter
//        armEncoderTalon.setSensorPhase(true);

        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        // PID
        armMotor.config_kF(0, Constants.Arm.FF); //feed forward
        armMotor.config_kP(0, Constants.Arm.P); //proportional
        armMotor.config_kI(0,Constants.Arm.I);
        armMotor.config_IntegralZone(0,200);

        // soft limits
        armMotor.configReverseSoftLimitEnable(true);
        armMotor.configReverseSoftLimitThreshold(Constants.Arm.MIN_POS);
        armMotor.configForwardSoftLimitEnable(true);
        armMotor.configForwardSoftLimitThreshold(Constants.Arm.MAX_POS);
    }

    // getting the ticks from the encoders.
    public double getPosition() {
        return armMotor.getSelectedSensorPosition(); // 4096
    }

    // getting the ticks from the encoders.
    public double getTargetPosition() {
        return armTargetPos; // 4096
    }

    // getting the ticks from the encoders.
    public void move(double percent) {
        if( (percent > 0 && getPosition() < Constants.Arm.MAX_POS) ||
            (percent < 0 && getPosition() > Constants.Arm.MIN_POS)
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
        return Math.abs(getPosition() - armTargetPos);
        //return armMotor.getClosedLoopError();
    }

    // limits
    public void currentLimitingEnable(boolean enableLimit) {
        armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(enableLimit, MAX_SUPPLY_CURRENT_AMPS,MAX_STATOR_CURRENT_AMPS, 0));
    }

    public void setArmTargetPos(double pos) {
        armTargetPos = pos;
    }

    public boolean isFinished() {
       // return Math.abs(PIDError()) < Constants.Arm.PID_TOLERANCE;
        return  PIDError() < Constants.Arm.PID_TOLERANCE;
    }

    public boolean getArmManualFlag() { return armManualFlag;}

    public void setMotionMagic(double position, double arbitraryFeedForward) {
        armMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    }

    public void setCruiseVelocity(int ticksPer100ms) {
        armMotor.configMotionCruiseVelocity(ticksPer100ms);
    }
    public void configF(double f) {
        armMotor.config_kF(0, f);
    }


    private void configArmTalon() {
        armMotor.configFactoryDefault();
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armMotor.enableVoltageCompensation(true);
        armMotor.setSensorPhase(true);
        armMotor.setInverted(true);
        armMotor.setNeutralMode(NeutralMode.Brake);

        armMotor.config_kP(0, Constants.Arm.P);
        armMotor.config_kI(0, Constants.Arm.I);
        armMotor.config_kD(0, Constants.Arm.D);
        armMotor.config_kF(0, Constants.Arm.F);
        armMotor.configMotionCruiseVelocity(Constants.Arm.MAX_VELOCITY);
        armMotor.configMotionAcceleration(Constants.Arm.MAX_ACCELERATION);
        armMotor.configNominalOutputForward(0);
        armMotor.configNominalOutputReverse(0);
        armMotor.configPeakOutputForward(1);
        armMotor.configPeakOutputReverse(-1);

        armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        armMotor.configForwardSoftLimitThreshold(Constants.Arm.MAX_POS);
        armMotor.configReverseSoftLimitEnable(true);
        armMotor.configReverseSoftLimitThreshold(Constants.Arm.MIN_POS);
        armMotor.overrideLimitSwitchesEnable(false);
        armMotor.configForwardSoftLimitEnable(true);
    }
}
