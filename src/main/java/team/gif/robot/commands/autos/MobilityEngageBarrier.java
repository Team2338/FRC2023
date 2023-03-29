package team.gif.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.lib.RobotTrajectory;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.autos.lib.SetInitialHeading;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;

import java.util.HashMap;

public class MobilityEngageBarrier extends SequentialCommandGroup {

    public MobilityEngageBarrier() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("PlaceMobEngage Barrier", 2.0, 1.5); // 1.8 1.2
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("goHome", new ParallelCommandGroup(
            new SetArmPosition(Constants.Arm.STAGE_POS),
            new ArmIn(),
            new SetElevatorPosition(Constants.Elevator.STAGE_POS)));

        FollowPathWithEvents trajectoryWithEvents = new FollowPathWithEvents(
            RobotTrajectory.getInstance().baseSwerveCommand(trajectory),
            trajectory.getMarkers(),
            eventMap
        );

        addCommands(
            new SetInitialHeading(trajectory),
            trajectoryWithEvents,
            new EngageFromFarSide()
        );
    }
}
