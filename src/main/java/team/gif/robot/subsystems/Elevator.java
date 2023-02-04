package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Elevator extends SubsystemBase {

    private static Elevator instance;

    private final WPI_TalonSRX elevatorMotor;

    public Elevator() {
        elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_ID);
        configLift(elevatorMotor);

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
            elevatorMotor.set(percent);
//        }
//        else
//            elevatorMotor.set(0);

    }

    public void zeroEncoder() {
        elevatorMotor.setSelectedSensorPosition(0);
    }

    private void configLift(TalonSRX talon) {
        talon.configFactoryDefault();
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        talon.enableVoltageCompensation(true);
        talon.setSensorPhase(true);
        talon.setInverted(false);
        talon.setNeutralMode(NeutralMode.Brake);

        talon.config_kP(0, Constants.Elevator.P);
        talon.config_kI(0, Constants.Elevator.I);
        talon.config_kD(0, Constants.Elevator.D);
        talon.config_kF(0, Constants.Elevator.F);
        talon.configMotionCruiseVelocity(Constants.Elevator.MAX_VELOCITY);
        talon.configMotionAcceleration(Constants.Elevator.MAX_ACCELERATION);
        talon.configNominalOutputForward(0);
        talon.configNominalOutputReverse(0);
        talon.configPeakOutputForward(1);
        talon.configPeakOutputReverse(-1);

        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        talon.configForwardSoftLimitThreshold(Constants.Elevator.MAX_POS);
        talon.configReverseSoftLimitThreshold(Constants.Elevator.MIN_POS);
        talon.overrideLimitSwitchesEnable(false);
        talon.configForwardSoftLimitEnable(true);
        talon.configReverseSoftLimitEnable(true);
//        talon.configClearPositionOnLimitR(true, 0);
    }
}
