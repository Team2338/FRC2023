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
public class Arm extends SubsystemBase {

    public static WPI_TalonSRX armMotor;
    public boolean armManualFlag = false;
    private double armTargetPos;

    private static final int MAX_SUPPLY_CURRENT_AMPS = 20;
    private static final int MAX_STATOR_CURRENT_AMPS = 90;
    public static DigitalInput armGamePieceSensor;

    public Arm() {
        armMotor = new WPI_TalonSRX(RobotMap.ARM_MOTOR);
        configArmTalon();

        armGamePieceSensor = new DigitalInput(9);
    }

    public void move(double percent) {
        if (Robot.oi.aux.getHID().getRightStickButton()) {
            armMotor.configReverseSoftLimitEnable(false);
//            armMotor.configReverseSoftLimitThreshold(0);
        } else {
            armMotor.configReverseSoftLimitEnable(true);
//-            armMotor.configReverseSoftLimitThreshold(Constants.Arm.MIN_POS);
        }

        // soft limits will keep the robot arm in allowable range
        armMotor.set(percent);
    }

    /**
     * Use PID to move the arm to a position
     */
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

    public void currentLimitingEnable(boolean enableLimit) {
        armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(enableLimit, MAX_SUPPLY_CURRENT_AMPS,MAX_STATOR_CURRENT_AMPS, 0));
    }

    public boolean isFinished() {
        return Math.abs(PIDError()) < Constants.Arm.PID_TOLERANCE;
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

    public void configI(double i) {
        armMotor.config_kI(0, i);
    }

    public void resetI() {
        armMotor.setIntegralAccumulator(0);
    }

    public double getI() {
        return armMotor.getIntegralAccumulator(0);
    }

    public double degreesToPos(double deg) { return deg * Constants.Arm.TICKS_PER_DEGREE - Constants.Arm.ZERO_OFFSET_TICKS;}

    /**
     *
     * @param degrees
     * @return degrees in units of ticks
     */
    public double degreesToTicks(double degrees) { return degrees * Constants.Arm.TICKS_PER_DEGREE;}

    public boolean getSensor() {
        return armGamePieceSensor.get();
    }

    public void configPeakOutputForward(double output) {
        armMotor.configPeakOutputForward(output);
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
        armMotor.config_kI(0,Constants.Arm.I_GT_45);
        armMotor.config_kD(0,Constants.Arm.D);
        armMotor.config_IntegralZone(0,314);

        // soft limits
        armMotor.configReverseSoftLimitEnable(true);
        armMotor.configReverseSoftLimitThreshold(Constants.Arm.MIN_POS);
        armMotor.configForwardSoftLimitEnable(true);
        armMotor.configForwardSoftLimitThreshold(Constants.Arm.MAX_POS);
        armMotor.configNominalOutputForward(0);     // nominal = minimum
        armMotor.configNominalOutputReverse(0);     // nominal = minimum
        armMotor.configPeakOutputReverse(Constants.Arm.PEAK_OUTPUT_REVERSE);     // use max 50% power
        armMotor.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT_FORWARD);      // use max 50% power
        armMotor.configClosedloopRamp(0.25); // time in seconds to get to peak output power

        // additional Motions Magic settings
        // *** arm is not currently using motion magic ***
        //armMotor.configMotionCruiseVelocity(Constants.Arm.MAX_VELOCITY);
        //armMotor.configMotionAcceleration(Constants.Arm.MAX_ACCELERATION);
    }
}
