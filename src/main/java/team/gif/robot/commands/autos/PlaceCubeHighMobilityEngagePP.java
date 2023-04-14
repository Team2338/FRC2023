package team.gif.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.lib.RobotTrajectory;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.collector.CollectorCollect;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;

import java.util.HashMap;

public class PlaceCubeHighMobilityEngagePP extends SequentialCommandGroup {

    public PlaceCubeHighMobilityEngagePP() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("1 GP Mobility Engage", 1.1, 1.5); // 1.8 1.2
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("goStage", new ParallelCommandGroup(
                new ArmIn(), // arm in just in case gravity pulled it out
                new AutoStagePos()));

        FollowPathWithEvents trajectoryWithEvents = new FollowPathWithEvents(
            RobotTrajectory.getInstance().baseSwerveCommand(trajectory),
            trajectory.getMarkers(),
            eventMap
        );

        addCommands(
            new SetArmPosition(Constants.Arm.STAGE_POS),
            new ParallelCommandGroup(
                    new SetArmPosition(Constants.Arm.PLACE_CUBE_HIGH_POS, Constants.Arm.PEAK_OUTPUT_FORWARD_CUBE_HIGH_POS),
                    new SetElevatorPosition(Constants.Elevator.PLACE_CUBE_HIGH_POS)
            ),
            new CollectorEject().withTimeout(0.15),
            trajectoryWithEvents,
            new EngageFromFarSide()
        );
    }
}
