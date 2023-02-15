package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

/*
 * 0 position is approx compass heading of 350
 *    (offsets are used in constants to take into account straight up being non-zero)
 * 0 degrees is considered straight up
 */

public class Arm extends SubsystemBase {

    public static WPI_TalonSRX armMotor;
    public boolean armManualFlag = false;
    private double armTargetPos;

    private static final int MAX_SUPPLY_CURRENT_AMPS = 20;
    private static final int MAX_STATOR_CURRENT_AMPS = 90;

    public Arm() {
        armMotor = new WPI_TalonSRX(RobotMap.ARM_MOTOR);
        configArmTalon();
    }

    public void move(double percent) {
        if( (percent > 0 && getPosition() < Constants.Arm.MAX_POS) ||
                (percent < 0 && getPosition() > Constants.Arm.MIN_POS)
        ) {
            armMotor.set(percent);
        }
        else
            armMotor.set(0);
    }

    public void PIDMove() {
        armMotor.set(ControlMode.Position, armTargetPos);
    }

    public double getPosition() {
        return armMotor.getSelectedSensorPosition(); // 4096
    }

    public double getPositionDegrees() {
        return (armMotor.getSelectedSensorPosition() - Constants.Arm.ZERO_OFFSET_TICKS ) / Constants.Arm.TICKS_PER_DEGREE; // 4096
    }
    
    public void setTargetPosition(double pos) {
        armTargetPos = pos;
    }

    public double getTargetPosition() {
        return armTargetPos; // 4096
    }

    public double getOutput(){
        return armMotor.getMotorOutputPercent();
    }

    public double PIDError(){
        return getPosition() - armTargetPos;
    }
    // limits

    public void currentLimitingEnable(boolean enableLimit) {
        armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(enableLimit, MAX_SUPPLY_CURRENT_AMPS,MAX_STATOR_CURRENT_AMPS, 0));
    }

    public boolean isFinished() {
        return Math.abs(PIDError()) < Constants.Arm.PID_TOLERANCE;
//        return  PIDError() < Constants.Arm.PID_TOLERANCE;
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

    public void configP(double p) {
        armMotor.config_kP(0, p);
    }

    private void configArmTalon() {
        armMotor.configFactoryDefault();

        //general settings
        armMotor.setNeutralMode(NeutralMode.Brake); // setting to brake mode
        armMotor.setInverted(true);
        armMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 20);
        armMotor.enableCurrentLimit(false);
        armMotor.setSensorPhase(true);
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        armMotor.configVoltageCompSaturation(12); // normalize full power to 12 volts
        armMotor.enableVoltageCompensation(true); // enables VoltageCompSaturation above

        // PID
        armMotor.config_kF(0, Constants.Arm.FF); // feed forward
        armMotor.config_kP(0, Constants.Arm.P); // proportional
        armMotor.config_kI(0,Constants.Arm.I);
        armMotor.config_kD(0,Constants.Arm.D);
        armMotor.config_IntegralZone(0,200);

        // soft limits
        armMotor.configReverseSoftLimitEnable(true);
        armMotor.configReverseSoftLimitThreshold(Constants.Arm.MIN_POS);
        armMotor.configForwardSoftLimitEnable(true);
        armMotor.configForwardSoftLimitThreshold(Constants.Arm.MAX_POS);
        armMotor.configNominalOutputForward(0);     // nominal = minimum
        armMotor.configNominalOutputReverse(0);     // nominal = minimum
        armMotor.configPeakOutputReverse(-0.5);     // use max 50% power
        armMotor.configPeakOutputForward(0.5);      // use max 50% power
        armMotor.configClosedloopRamp(1); // time in seconds to get to peak output power

        // additional Motions Magic settings
        armMotor.configMotionCruiseVelocity(Constants.Arm.MAX_VELOCITY);
        armMotor.configMotionAcceleration(Constants.Arm.MAX_ACCELERATION);
    }
}
