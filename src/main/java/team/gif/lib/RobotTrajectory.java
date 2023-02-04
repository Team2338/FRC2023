package team.gif.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class RobotTrajectory {
    public RobotTrajectory() {

    }

    private static RobotTrajectory instance = null;

    public static RobotTrajectory getInstance() {
        if(instance == null) {
            instance = new RobotTrajectory();
        }
        return instance;
    }

    public TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
    )
        .setKinematics(Constants.Drivetrain.kDriveKinematics);

    public SwerveControllerCommand swerveControllerCommand(Trajectory trajectory) {
        ProfiledPIDController kThetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
            new TrapezoidProfile.Constraints(
                Constants.ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, Constants.ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared
            ));

        kThetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand sCC = new SwerveControllerCommand(
            trajectory,
            Robot.swervetrain::getPose,
            Constants.Drivetrain.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            kThetaController,
            Robot.swervetrain::setModuleStates,
            Robot.swervetrain
        );

        Robot.swervetrain.resetOdometry(trajectory.getInitialPose());

        return sCC;
    }
}