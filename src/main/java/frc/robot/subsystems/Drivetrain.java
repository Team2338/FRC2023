package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
    private static TalonSRX rightSide;
    private static TalonSRX leftSide;
    private static DifferentialDrive differentialDrive;
    public Drivetrain(boolean isLInv, boolean isRInv) {
        super();

        rightSide = new WPI_TalonSRX(RobotMap.RIGHT_DRIVETRAIN);
        leftSide = new WPI_TalonSRX(RobotMap.LEFT_DRIVETRAIN);

        rightSide.configFactoryDefault();
        leftSide.configFactoryDefault();

        rightSide.setNeutralMode(NeutralMode.Brake);
        leftSide.setNeutralMode(NeutralMode.Brake);

        leftSide.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 20);
        rightSide.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 20);

        leftSide.setInverted(isLInv);
        rightSide.setInverted(isRInv);

        differentialDrive = new DifferentialDrive((MotorController) rightSide, (MotorController) leftSide);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setDeadband(0.02);

        resetEncoder();

//    pigeon = new Pigeon();
//    pigeon.resetPigeonPosition();
    }

    public void resetEncoder() {
        leftSide.setSelectedSensorPosition(0);
        rightSide.setSelectedSensorPosition(0);
    }

    public void driveArcade(double spd, double rot) {
        differentialDrive.arcadeDrive(spd, rot);
    }

    public void driveTank(double lS, double rS) {
        differentialDrive.tankDrive(lS,rS);
    }
}
