package team.gif.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import team.gif.robot.Robot;
import team.gif.lib.RobotTrajectory;
import java.util.List;

public class SwervePOC extends SequentialCommandGroup {
    private Trajectory trajectory;

    public Command go() {
        trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.0, 0.0, new Rotation2d()),
                new Pose2d(30.0, 0.0, new Rotation2d())
            ),
            RobotTrajectory.getInstance().trajectoryConfig
        );

        SwerveControllerCommand sCC = new RobotTrajectory().swerveControllerCommand(trajectory);

        return sCC.andThen(() -> Robot.swervetrain.drive(0.0, 0.0, 0.0, false));
    }

    public SwervePOC() {
        addCommands(
            new InstantCommand(() -> Robot.swervetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()))),
            go(),
            new InstantCommand(() -> Robot.swervetrain.stopModules())
        );
    }
}
