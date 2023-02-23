package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.Pigeon;

public class Drivetrain extends SubsystemBase {
    private static TalonSRX rightSideOne;
    private static TalonSRX rightSideTwo;
    private static TalonSRX leftSideOne;
    private static TalonSRX leftSideTwo;

    private static MotorControllerGroup leftMotors;
    private static MotorControllerGroup rightMotors;
    private static DifferentialDrive drive;

    private static Pigeon pigeon;

    public Drivetrain(boolean isLInv, boolean isRInv) {
        super();

        rightSideOne = new WPI_TalonSRX(RobotMap.RIGHT_DRIVETRAIN_ONE);
        rightSideTwo = new WPI_TalonSRX(RobotMap.RIGHT_DRIVETRAIN_TWO);
        leftSideOne = new WPI_TalonSRX(RobotMap.LEFT_DRIVETRAIN_ONE);
        leftSideTwo = new WPI_TalonSRX(RobotMap.LEFT_DRIVETRAIN_TWO);

        leftMotors = new MotorControllerGroup((MotorController) leftSideOne, (MotorController) leftSideTwo);
        rightMotors = new MotorControllerGroup((MotorController) rightSideOne, (MotorController) rightSideTwo);
        drive = new DifferentialDrive(leftMotors, rightMotors);

        leftSideOne.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 20);
        rightSideOne.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 20);

        rightSideOne.setNeutralMode(NeutralMode.Brake);
        rightSideTwo.setNeutralMode(NeutralMode.Brake);
        leftSideOne.setNeutralMode(NeutralMode.Brake);
        leftSideTwo.setNeutralMode(NeutralMode.Brake);

        // turn off the drive train watchdog - otherwise it outputs unnecessary errors on the console
        drive.setSafetyEnabled(false);

        rightSideOne.configFactoryDefault();
        leftSideOne.configFactoryDefault();

        leftSideOne.setInverted(false);
        leftSideTwo.setInverted(false);
        rightSideOne.setInverted(false);
        rightSideTwo.setInverted(false);

        leftSideOne.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
        leftSideTwo.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
        rightSideOne.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
        rightSideTwo.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);

        leftSideOne.configVelocityMeasurementWindow(32);
        leftSideTwo.configVelocityMeasurementWindow(32);
        rightSideOne.configVelocityMeasurementWindow(32);
        rightSideTwo.configVelocityMeasurementWindow(32);

//        differentialDrive.setDeadband(0.02 : .05);

        //pigeon = new Pigeon();
        Robot.pigeon.resetPigeonPosition();
    }

    public void resetEncoder() {
        leftSideOne.setSelectedSensorPosition(0);
        rightSideOne.setSelectedSensorPosition(0);
    }

    public void driveArcade(double spd, double rot) {
        drive.arcadeDrive(spd, rot);
    }

    public void driveTank(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed,rightSpeed);
    }
}
