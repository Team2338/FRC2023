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

    /**
     * Moves the elevator with a percent output
     * @param percent percent of max speed to run the elevator motor
     */
    public void move(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Holds the elevator at its given position with PID
     */
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

    /**
     * Get the position of the elevator in encoder ticks
     * @return the position of the elevator in encoder ticks
     */
    public double getPosition() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    /**
     * Get the position of the elevator in inches
     * @return the position of the elevator in inches
     */
    public double getPositionInches() {
        return (elevatorMotor.getSelectedSensorPosition() + Constants.Elevator.ZERO_OFFSET_TICKS) / Constants.Elevator.EL_TICKS_PER_INCH;
    }

    /**
     * Convert a position in inches to encoder ticks
     * @param inches the position in inches
     * @return the position in encoder ticks
     */
    public double inchesToPos(int inches) {
        return inches * Constants.Elevator.EL_TICKS_PER_INCH - Constants.Elevator.ZERO_OFFSET_TICKS;
    }

    /**
     *
     * @param inches
     * @return Inches in units of ticks
     */
    public double inchesToTicks(int inches) {
        return inches * Constants.Elevator.EL_TICKS_PER_INCH;
    }

    /**
     * Get the target elevator position
     * @return the target elevator position
     */
    public double getTargetPosition() {
        return elevatorTargetPos;
    }

    /**
     * Get the PID error on the elevator position
     * @return the PID error on the elevator position
     */
    public double PIDError() {
        return Math.abs(getPosition() - elevatorTargetPos);
    }

    /**
     * Set the velocity of the elevator as a percent output
     * @param percent percent of max speed to run the elevator motor
     */
    public void setPercentOutput(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Sets the elevator to a given position with Motion Magic
     * @param position the position to set the elevator to
     */
    public void setMotionMagic(double position) {
        elevatorMotor.set(ControlMode.MotionMagic, position);
    }

    /**
     * Sets the elevator to a given position with Motion Magic and an arbitrary feed forward
     * @param position the position to set the elevator to
     * @param arbitraryFeedForward the arbitrary feed forward to use
     */
    public void setMotionMagic(double position, double arbitraryFeedForward) {
        elevatorMotor.selectProfileSlot(0,0);
        elevatorMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
    }

    /**
     * Sets the cruise velocity of the elevator in ticks per 100ms
     * @param ticksPer100ms the cruise velocity of the elevator in ticks per 100ms
     */
    public void setCruiseVelocity(int ticksPer100ms) {
        elevatorMotor.configMotionCruiseVelocity(ticksPer100ms);
    }

    /**
     * Sets the target elevator position
     * @param pos the target elevator position
     */
    public void setElevatorTargetPos(double pos) {
        elevatorTargetPos = pos;
    }

    /**
     * Sets the elevator motor's feed forward
     * @param f the feed forward to set
     */
    public void configF(double f) {
        elevatorMotor.config_kF(0, f);
    }

    /**
     * Sets the forward limit switch to be normally open or normally closed
     * @return true if the limit switch is closed
     */
    public boolean getFwdLimit() {
        return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * Sets the reverse limit switch to be normally open or normally closed
     * @return true if the limit switch is closed
     */
    public boolean getRevLimit() {
        return elevatorMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    /**
     * Checks if the elevator is at its target position
     * @return true if the elevator is at its target position
     */
    public boolean isFinished() {
        return  Math.abs(PIDError()) < Constants.Elevator.PID_TOLERANCE;
    }

    /**
     * Checks if the elevator is at its target position
     * @return true if the elevator is at its target position
     */
    public double getOutputVoltage() {
        return elevatorMotor.getMotorOutputVoltage();
    }

    /**
     * Gets the percent output of the elevator motor
     * @return the percent output of the elevator motor
     */
    public double getOutputPercent() {
        return elevatorMotor.getMotorOutputPercent();
    }

    /**
     * Gets the velocity of the elevator in ticks per 100ms
     * @return the velocity of the elevator in ticks per 100ms
     */
    public double getVelTPS() {
        return elevatorMotor.getSelectedSensorVelocity() * 10.0;
    }

    /**
     * Gets the current of the elevator motor
     * @return the current of the elevator motor
     */
    public double getCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    /**
     * Sets the reverse soft limit of the elevator
     * @param engage whether to engage the soft limit
     */
    public void enableLowerSoftLimit(boolean engage) {
        elevatorMotor.configReverseSoftLimitEnable(engage);
    }

    /**
     * Zeroes the elevator encoder
     */
    public void zeroEncoder() {
        elevatorMotor.setSelectedSensorPosition(0);
    }

    /**
     * Configures the elevator Talon
     */
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
