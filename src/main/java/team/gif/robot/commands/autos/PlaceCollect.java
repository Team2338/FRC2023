package team.gif.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import team.gif.lib.RobotTrajectory;
import team.gif.robot.Robot;

import java.util.List;

public class PlaceCollect extends SequentialCommandGroup {
    private Trajectory trajectory;

    public Command go() {
        trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d()),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(2, 0)
                ),
                new Pose2d(3, 0, new Rotation2d(Units.degreesToRadians(180))),
                RobotTrajectory.getInstance().trajectoryConfig
        );

        SwerveControllerCommand scc = new RobotTrajectory().swerveControllerCommand(trajectory);

        return scc.andThen(() -> Robot.swervetrain.drive(0, 0, 0, false));
    }
}
