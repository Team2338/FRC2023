package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.Pigeon;
import team.gif.robot.subsystems.drivers.SwerveModuleCANCoder;

public class SwerveDrivetrain extends SubsystemBase {
    public static SwerveModuleCANCoder fL;
    public static SwerveModuleCANCoder fR;
    public static SwerveModuleCANCoder rR;
    public static SwerveModuleCANCoder rL;

    private static TalonSRX pigMotor;
    private static Pigeon pig;

    private static SwerveDriveOdometry odometry;

    /**
     * Constructor for swerve drivetrain using 4 swerve modules using NEOs to drive and TalonSRX to control turning
     */
    public SwerveDrivetrain() {
        super();

        fL = new SwerveModuleCANCoder(
                RobotMap.FRONT_LEFT_DRIVE_MOTOR_PORT,
                RobotMap.FRONT_LEFT_TURNING_MOTOR_PORT,
                false,
                false,
                true,
                Constants.Drivetrain.FRONT_LEFT_OFFSET,
                RobotMap.FRONT_LEFT_CANCODER,
                Constants.ModuleConstants.DrivetrainPID.frontLeftFF,
                Constants.ModuleConstants.DrivetrainPID.frontLeftP
        );

        fR = new SwerveModuleCANCoder(
                RobotMap.FRONT_RIGHT_DRIVE_MOTOR_PORT,
                RobotMap.FRONT_RIGHT_TURNING_MOTOR_PORT,
                false,
                true,
                true,
                Constants.Drivetrain.FRONT_RIGHT_OFFSET,
                RobotMap.FRONT_RIGHT_CANCODER,
                Constants.ModuleConstants.DrivetrainPID.frontRightFF,
                Constants.ModuleConstants.DrivetrainPID.frontRightP
        );

        rR = new SwerveModuleCANCoder(
                RobotMap.REAR_RIGHT_DRIVE_MOTOR_PORT,
                RobotMap.REAR_RIGHT_TURNING_MOTOR_PORT,
                false,
                true,
                true,
                Constants.Drivetrain.REAR_RIGHT_OFFSET,
                RobotMap.REAR_RIGHT_CANCODER,
                Constants.ModuleConstants.DrivetrainPID.rearRightFF,
                Constants.ModuleConstants.DrivetrainPID.rearRightP
        );

        rL = new SwerveModuleCANCoder(
                RobotMap.REAR_LEFT_DRIVE_MOTOR_PORT,
                RobotMap.REAR_LEFT_TURNING_MOTOR_PORT,
                false,
                false,
                true,
                Constants.Drivetrain.REAR_LEFT_OFFSET,
                RobotMap.REAR_LEFT_CANCODER,
                Constants.ModuleConstants.DrivetrainPID.rearLeftFF,
                Constants.ModuleConstants.DrivetrainPID.rearLeftP
        );

//        resetEncoders();
        pigMotor = new TalonSRX(RobotMap.PIGEON);
        pig = new Pigeon(pigMotor);
        odometry = new SwerveDriveOdometry(Constants.Drivetrain.DRIVE_KINEMATICS, pig.getRotation2d(), getPosition(), new Pose2d(0, 0, new Rotation2d(0)));

        resetHeading();
        resetDriveEncoders();

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
        swerveTab.addDouble("FL_Rotation", fL::getRawHeading);
        swerveTab.addDouble("FR_Rotation", fR::getRawHeading);
        swerveTab.addDouble("RL_Rotation", rL::getRawHeading);
        swerveTab.addDouble("RR_Rotation", rR::getRawHeading);
    }

    @Override
    public void periodic() {
        odometry.update(
                new Rotation2d().fromDegrees(pig.get360Heading()), //TODO: Check getHeading Function
                getPosition()
        );
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pig.getRotation2d(), new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()}, pose);
    }

    public void drive(double x, double y, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(
                        fieldRelative ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, pig.getRotation2d())
                                : new ChassisSpeeds(x, y, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Drivetrain.MAX_SPEED_METERS_PER_SECOND
        );

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);
    }


    /**
     * Set the desired states for each of the 4 swerve modules using a SwerveModuleState array
     * @param desiredStates SwerveModuleState array of desired states for each of the modules
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Constants.Drivetrain.MAX_SPEED_METERS_PER_SECOND
        );

        fL.setDesiredState(desiredStates[0]);
        fR.setDesiredState(desiredStates[1]);
        rL.setDesiredState(desiredStates[2]);
        rR.setDesiredState(desiredStates[3]);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a ChassisSpeeds class
     * @param chassisSpeeds Field Relative ChassisSpeeds to apply to wheel speeds
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Drivetrain.MAX_SPEED_METERS_PER_SECOND
        );

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Reset the position of each of the wheels so that they all are pointing straight forward
     */
    public void resetEncoders() {
        fL.resetEncoders();
        fR.resetEncoders();
        rL.resetEncoders();
        rR.resetEncoders();
    }

    /**
     * Reset the pigeon heading
     */
    public void resetHeading() {
        pig.resetPigeonPosition();
    }


    /**
     * Get the pigeon heading
     * @return The pigeon heading in degrees
     */
    public Rotation2d getHeading() {
        return pig.getRotation2d();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void stopModules() {
        fL.stop();
        fR.stop();
        rR.stop();
        rL.stop();
    }

    public SwerveModulePosition[] getPosition() {
        return new SwerveModulePosition[] {fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()};
    }

    public void resetDriveEncoders() {
        fL.resetDriveEncoders();
        fR.resetDriveEncoders();
        rL.resetDriveEncoders();
        rR.resetDriveEncoders();
    }

    public double getRobotHeading() {
        return pig.getCompassHeading();
    }

    public Pose2d getRobotPose() {
        return odometry.getPoseMeters();
    }
}
