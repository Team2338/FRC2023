package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.Pigeon;
import team.gif.robot.subsystems.drivers.SwerveModule;
import team.gif.robot.subsystems.drivers.SwerveModuleCANCoder;

public class SwerveDrivetrain extends SubsystemBase {
    public static SwerveModule fL;
    public static SwerveModule fR;
    public static SwerveModuleCANCoder rR;
    public static SwerveModule rL;

    private static TalonSRX pigMotor;
    private static Pigeon pig;

    private static SwerveDriveOdometry odometry;

    /**
     * Constructor for swerve drivetrain using 4 swerve modules using NEOs to drive and TalonSRX to control turning
     */
    public SwerveDrivetrain() {
        super();

        fL = new SwerveModule(
                RobotMap.FRONT_LEFT_DRIVE_MOTOR_PORT,
                RobotMap.FRONT_LEFT_TURNING_MOTOR_PORT,
                false,
                true,
                true,
                Constants.Drivetrain.kFrontLeftOffset,
                0.03,
                0.5
        );

        fR = new SwerveModule(
                RobotMap.FRONT_RIGHT_DRIVE_MOTOR_PORT,
                RobotMap.FRONT_RIGHT_TURNING_MOTOR_PORT,
                false,
                false,
                true,
                Constants.Drivetrain.kFrontRightOffset,
                0.03,
                0.5
        );

        rR = new SwerveModuleCANCoder(
                RobotMap.REAR_RIGHT_DRIVE_MOTOR_PORT,
                RobotMap.REAR_RIGHT_TURNING_MOTOR_PORT,
                false,
                false,
                true,
                Constants.Drivetrain.kRearRightOffset,
                RobotMap.REAR_RIGHT_CANCODER,
                0.02,
                0.5
        );

        rL = new SwerveModule(
                RobotMap.REAR_LEFT_DRIVE_MOTOR_PORT,
                RobotMap.REAR_LEFT_TURNING_MOTOR_PORT,
                false,
                false,
                true,
                Constants.Drivetrain.kRearLeftOffset,
                0.02,
                0.5
        );

//        resetEncoders();
        pigMotor = new TalonSRX(RobotMap.PIGEON);
        pig = new Pigeon(pigMotor);
        odometry = new SwerveDriveOdometry(Constants.Drivetrain.kDriveKinematics, pig.getRotation2d(), getPosition(), new Pose2d(0, 0, new Rotation2d(0)));

        resetHeading();
        resetDriveEncoders();

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
                Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(
                        fieldRelative ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, pig.getRotation2d())
                                : new ChassisSpeeds(x, y, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond
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
                desiredStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond
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
        SwerveModuleState[] swerveModuleStates = Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Drivetrain.kMaxSpeedMetersPerSecond
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
}
