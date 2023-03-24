package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class Elevator extends SubsystemBase {
    public final WPI_TalonSRX elevatorMotor;

    public boolean elevatorManualFlag = false;
    private double elevatorTargetPos;

    public Elevator() {
        elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_ID);
        configElevatorTalon();
        zeroEncoder();
    }

    public void move(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    public void PIDHold() {
        elevatorMotor.selectProfileSlot(1,0);
        // the elevator needs a different kF when it is lower to the ground, otherwise it doesn't stay at the position
        if( elevatorTargetPos < Constants.Elevator.PLACE_CUBE_MID_POS) {
            elevatorMotor.config_kF(1, Constants.Elevator.F_HOLD_LOW);
        }
        else
            elevatorMotor.config_kF(1, Constants.Elevator.F_HOLD);

        elevatorMotor.set(ControlMode.Position, elevatorTargetPos); // closed loop position control
    }

    public double getPosition() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public double getPositionInches() {
        return (elevatorMotor.getSelectedSensorPosition() + Constants.Elevator.ZERO_OFFSET_TICKS) / Constants.Elevator.EL_TICKS_PER_INCH;
    }

    public double convertInchesToPos(int inches ){
        return inches * Constants.Elevator.EL_TICKS_PER_INCH - Constants.Elevator.ZERO_OFFSET_TICKS;
    }

    public double getTargetPosition() {
        return elevatorTargetPos;
    }

    public double PIDError() {
        return Math.abs(getPosition() - elevatorTargetPos);
    }

    public void setPercentOutput(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setMotionMagic(double position) {
        elevatorMotor.set(ControlMode.MotionMagic, position);
    }

    public void setMotionMagic(double position, double arbitraryFeedForward) {
        elevatorMotor.selectProfileSlot(0,0);
        elevatorMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    }

    public void setCruiseVelocity(int ticksPer100ms) {
        elevatorMotor.configMotionCruiseVelocity(ticksPer100ms);
    }

    public void setElevatorTargetPos(double pos) {
        elevatorTargetPos = pos;
    }

    public void configF(double f) {
        elevatorMotor.config_kF(0, f);
    }

    public boolean getFwdLimit() {
        return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getRevLimit() {
        return elevatorMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean isFinished() {
        return  Math.abs(PIDError()) < Constants.Elevator.PID_TOLERANCE;
    }

    public double getOutputVoltage() {
        return elevatorMotor.getMotorOutputVoltage();
    }

    public double getOutputPercent() {
        return elevatorMotor.getMotorOutputPercent();
    }

    public double getVelTPS() {
        return elevatorMotor.getSelectedSensorVelocity() * 10.0;
    }

    public double getCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    public void enableLowerSoftLimit(boolean engage) {
        elevatorMotor.configReverseSoftLimitEnable(engage);
    }

    public void zeroEncoder() {
        elevatorMotor.setSelectedSensorPosition(0);
    }

    private void configElevatorTalon() {
        elevatorMotor.configFactoryDefault();
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        elevatorMotor.enableVoltageCompensation(true);
        elevatorMotor.setSensorPhase(true);
        elevatorMotor.setInverted(Robot.isCompBot ? true : false);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);

        elevatorMotor.config_kP(0, Constants.Elevator.P);
        elevatorMotor.config_kI(0, Constants.Elevator.I);
        elevatorMotor.config_kD(0, Constants.Elevator.D);
        elevatorMotor.config_kF(0, Constants.Elevator.F);

        elevatorMotor.config_kP(1, Constants.Elevator.P_HOLD);
        elevatorMotor.config_kI(1, Constants.Elevator.I_HOLD);
        elevatorMotor.config_kD(1, Constants.Elevator.D_HOLD);
        elevatorMotor.config_kF(1, Constants.Elevator.F_HOLD);

        elevatorMotor.configMotionCruiseVelocity(Constants.Elevator.MAX_VELOCITY);
        elevatorMotor.configMotionAcceleration(Constants.Elevator.MAX_ACCELERATION);
        elevatorMotor.configNominalOutputForward(0);
        elevatorMotor.configNominalOutputReverse(0);
        elevatorMotor.configPeakOutputForward(1);
        elevatorMotor.configPeakOutputReverse(-1);

        elevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        elevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        elevatorMotor.configForwardSoftLimitThreshold(Constants.Elevator.MAX_POS);
        elevatorMotor.configReverseSoftLimitThreshold(Constants.Elevator.MIN_POS);
        elevatorMotor.overrideLimitSwitchesEnable(false);
        elevatorMotor.configForwardSoftLimitEnable(true);
        elevatorMotor.configReverseSoftLimitEnable(true);
    }
}
