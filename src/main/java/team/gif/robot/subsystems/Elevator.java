package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Elevator extends SubsystemBase {

    private static Elevator instance;

    private final TalonSRX elevatorMotor;

    public Elevator() {
        elevatorMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR_ID);
        configElevatorTalon();

//        int absPos = elevatorMotor.getSensorCollection().getPulseWidthPosition();
//        absPos &= 0xFFF;
//        elevatorMotor.setSelectedSensorPosition(absPos);
//        elevatorMotor.setSelectedSensorPosition(0);
        // Soft Limits
        elevatorMotor.configReverseSoftLimitEnable(true);
        elevatorMotor.configReverseSoftLimitThreshold(Constants.Elevator.MIN_POS);
        elevatorMotor.configForwardSoftLimitEnable(true);
        elevatorMotor.configForwardSoftLimitThreshold(Constants.Elevator.MAX_POS);

        zeroEncoder();
    }
    
    public void setPercentOutput(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
//        lift.set(ControlMode.PercentOutput, percent);
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
        return Math.abs(elevatorMotor.getClosedLoopError()) < Constants.Elevator.ALLOWABLE_ERROR;
    }

    public double getElevatorClosedLoopError() {
        return elevatorMotor.getClosedLoopError();
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

    //public double getVelRPS() {
       // return getVelTPS() * Constants.Drivetrain.TPS_TO_RPS;
    //}

    public double getCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    public void enableLowerSoftLimit(boolean engage) {
        elevatorMotor.configReverseSoftLimitEnable(engage);
    }
//    public int getClosedLoopError() {
//        //return lift.getClosedLoopError();
//    }
    public void move(double percent) {
//        if( (percent > 0 && getPosition() < Constants.Elevator.MAX_POS) ||
//                (percent < 0 && getPosition() > Constants.Elevator.MIN_POS)
//        ) {
//-            elevatorMotor.set(percent);
            elevatorMotor.set(ControlMode.PercentOutput, percent);
//        }
//        else
//            elevatorMotor.set(0);

    }

    public void zeroEncoder() {
        elevatorMotor.setSelectedSensorPosition(0);
    }

    private void configElevatorTalon() {
        elevatorMotor.configFactoryDefault();
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
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
//        talon.configClearPositionOnLimitR(true, 0);
    }
}
