package team.gif.lib.path;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

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
        ProfiledPIDController kThetaController = new ProfiledPIDController(Constants.AutoConstants.P_THETA_CONTROLLER, 0, 0,
            new TrapezoidProfile.Constraints(
                Constants.ModuleConstants.MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND, Constants.ModuleConstants.MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
            ));

        kThetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand sCC = new SwerveControllerCommand(
            trajectory,
            Robot.swervetrain::getPose,
            Constants.Drivetrain.DRIVE_KINEMATICS,
            new PIDController(Constants.AutoConstants.PX_CONTROLLER, 0, 0),
            new PIDController(Constants.AutoConstants.PY_CONTROLLER, 0, 0),
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
     * @param subsystems All subsystems needed to run the auto; must also include {@link Robot#swervetrain}
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

    /**
     *
     * @param traj
     * @param isFirstPath
     * @return
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if(isFirstPath) {
                        Robot.swervetrain.resetOdometry(traj.getInitialPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        Robot.swervetrain::getPose,
                        Constants.Drivetrain.DRIVE_KINEMATICS,
                        new PIDController(Constants.AutoConstants.PX_CONTROLLER, 0, 0),
                        new PIDController(Constants.AutoConstants.PY_CONTROLLER, 0, 0),
                        new PIDController(Constants.AutoConstants.P_THETA_CONTROLLER, 0, 0),
                        Robot.swervetrain::setModuleStates,
                        Robot.swervetrain
                )
        );
    }

    public PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
        Robot.swervetrain.resetOdometry(trajectory.getInitialPose());

        PPSwerveControllerCommand swerveControllerCommand =
                new PPSwerveControllerCommand(
                        trajectory,
                        Robot.swervetrain::getPose,
                        Constants.Drivetrain.DRIVE_KINEMATICS,
                        new PIDController(SmartDashboard.getNumber("kPX", 5.0), 0, 0),
                        new PIDController(SmartDashboard.getNumber("kPY", 5.0), 0, 0),
                        new PIDController(SmartDashboard.getNumber("kPTheta", 3.7), 0, 0),
                        Robot.swervetrain::setModuleStates,
                        //true, <-- currently not working
                        Robot.swervetrain
                );

        return swerveControllerCommand;
    }
}