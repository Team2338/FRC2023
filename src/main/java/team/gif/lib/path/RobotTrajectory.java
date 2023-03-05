package team.gif.lib.path;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.SwerveDrivetrain;

import java.util.HashMap;

/**
 * Singleton class for creating a trajectory for a swerve bot
 * @author Rohan Cherukuri
 * @since 2/14/23
 */
public class RobotTrajectory {
    public RobotTrajectory() {}

    private static RobotTrajectory instance = null;

    public static RobotTrajectory getInstance() {
        if(instance == null) {
            instance = new RobotTrajectory();
        }
        return instance;
    }

    public TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
        Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
    )
        .setKinematics(Constants.Drivetrain.DRIVE_KINEMATICS);

    public SwerveControllerCommand swerveControllerCommand(Trajectory trajectory) {
        ProfiledPIDController kThetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
            new TrapezoidProfile.Constraints(
                Constants.ModuleConstants.MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND, Constants.ModuleConstants.MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
            ));

        kThetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand sCC = new SwerveControllerCommand(
            trajectory,
            Robot.swervetrain::getPose,
            Constants.Drivetrain.DRIVE_KINEMATICS,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            kThetaController,
            Robot.swervetrain::setModuleStates,
            Robot.swervetrain
        );

        Robot.swervetrain.resetOdometry(trajectory.getInitialPose());

        return sCC;
    }


    /**
     *
     * @param eventMap
     * @param subsystems
     * @return
     */
    public SwerveAutoBuilder buildConfig(HashMap<String, Command> eventMap, Subsystem... subsystems) {
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            Robot.swervetrain::getPose,
            Robot.swervetrain::resetOdometry,
            Constants.Drivetrain.DRIVE_KINEMATICS,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.5, 0, 0),
            Robot.swervetrain::setModuleStates,
            eventMap,
            subsystems
        );

        return autoBuilder;
    }
}