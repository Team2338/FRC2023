package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Elevator extends SubsystemBase {

    public final WPI_TalonSRX elevatorMotor;

    private double elevatorPos;

    public boolean elevatorManualFlag = false;

    /*
    * This class controls the elevator.
    *
    * 0 position is straight up.
    * */
    public Elevator() {
        elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_ID);
        configElevatorTalon();

        // Soft Limits
        elevatorMotor.configReverseSoftLimitEnable(true);
        elevatorMotor.configReverseSoftLimitThreshold(Constants.Elevator.MIN_POS);
        elevatorMotor.configForwardSoftLimitEnable(true);
        elevatorMotor.configForwardSoftLimitThreshold(Constants.Elevator.MAX_POS);

        zeroEncoder();
    }
    
    public void setPercentOutput(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setMotionMagic(double position) {
        elevatorMotor.set(ControlMode.MotionMagic, position);
    }

    public void setMotionMagic(double position, double arbitraryFeedForward) {
        elevatorMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    }

    public void setCruiseVelocity(int ticksPer100ms) {
        elevatorMotor.configMotionCruiseVelocity(ticksPer100ms);
    }

    public void PIDMove() {
        elevatorMotor.set(ControlMode.Position, elevatorPos); // closed loop position control
    }

    public void setElevatorTargetPos(double pos) {
        elevatorPos = pos;
    }

    public double PIDError() {
        return elevatorMotor.getClosedLoopError();
    }

    public void configF(double f) {
        elevatorMotor.config_kF(0, f);
    }

    public double getPosition() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public boolean getFwdLimit() {
        return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getRevLimit() {
        return elevatorMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean isFinished() {
        System.out.println( "            Error: " + elevatorMotor.getClosedLoopError());
        return Math.abs(elevatorMotor.getClosedLoopError()) < Constants.Elevator.PID_TOLERANCE;
    }

    public double getOutputVoltage() {
        return elevatorMotor.getMotorOutputVoltage();
    }

    public double getOutputCommand() {
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

    public void move(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    public void zeroEncoder() {
        elevatorMotor.setSelectedSensorPosition(0);
    }

    private void configElevatorTalon() {
        elevatorMotor.configFactoryDefault();
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        elevatorMotor.enableVoltageCompensation(true);
        elevatorMotor.setSensorPhase(true);
        elevatorMotor.setInverted(false);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);

        elevatorMotor.config_kP(0, Constants.Elevator.P);
        elevatorMotor.config_kI(0, Constants.Elevator.I);
        elevatorMotor.config_kD(0, Constants.Elevator.D);
        elevatorMotor.config_kF(0, Constants.Elevator.F);
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
