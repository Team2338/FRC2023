package team.gif.lib;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

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

    public AutoPIDConfig lowSpeedPIDConfig = new AutoPIDConfig( // designed for zephyr at 1.8 m/s and 1.2 m/s/s
            new PIDController(Constants.AutoConstants.PX_CONTROLLER, 0, 0),
            new PIDController(Constants.AutoConstants.PY_CONTROLLER, 0, 0),
            new PIDController(Constants.AutoConstants.P_THETA_CONTROLLER, 0, 0)
    );

    public AutoPIDConfig highSpeedPIDConfig = new AutoPIDConfig( // designed for zephyr at 3.0 m/s and 5.5 m/s/s
            new PIDController(Constants.AutoConstants.FAST_PX_CONTROLLER, 0, 0),
            new PIDController(Constants.AutoConstants.FAST_PY_CONTROLLER, 0, 0),
            new PIDController(Constants.AutoConstants.FAST_PY_CONTROLLER, 0, 0)
    );

    public PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory, AutoPIDConfig autoPIDConfig) {
        // rest odometry to the initial position of the path
//        Robot.pigeon.resetPigeonPosition( trajectory.getInitialHolonomicPose().getRotation().getDegrees());
//        Robot.swervetrain.resetOdometry(trajectory.getInitialHolonomicPose());
        PathPlannerTrajectory trajectory1 = PathPlannerTrajectory.transformTrajectoryForAlliance(
                trajectory, DriverStation.getAlliance());

        return new PPSwerveControllerCommand(
            trajectory1,
            Robot.swervetrain::getPose,
            Constants.Drivetrain.DRIVE_KINEMATICS,
            // TODO SwerveAuto can remove and add after PID constants are finalized and autos are running well
            //SA new PIDController(SmartDashboard.getNumber("kPX", 5.0), 0, 0),
            //SA new PIDController(SmartDashboard.getNumber("kPY", 5.0), 0, 0),
            //SA new PIDController(SmartDashboard.getNumber("kPTheta", 3.7), 0, 0),
            autoPIDConfig.xController,
            autoPIDConfig.yController,
            autoPIDConfig.thetaController,
            Robot.swervetrain::setModuleStates,
//            true, //<-- currently not working
            Robot.swervetrain
        );
    }
}