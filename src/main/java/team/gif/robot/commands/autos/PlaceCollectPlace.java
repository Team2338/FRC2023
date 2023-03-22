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
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.collector.WheelsOut;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmOut;

import java.util.HashMap;

public class PlaceCollectPlace extends SequentialCommandGroup {

    public PlaceCollectPlace() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Collect GP One Barrier", 1.8, 1.2);
        HashMap<String, Command> eventMap = new HashMap<>();

        FollowPathWithEvents trajectoryWithEvents = new FollowPathWithEvents(
                RobotTrajectory.getInstance().baseSwerveCommand(trajectory),
                trajectory.getMarkers(),
                eventMap
        );

        addCommands(
            new WheelsOut(),
            new SetArmPosition(Constants.Arm.STAGE_POS),
            new ParallelCommandGroup(
                new SetElevatorPosition(Constants.Elevator.PLACE_CONE_HIGH_POS),
                new SetArmPosition(Constants.Arm.PLACE_CONE_HIGH_POS, Constants.Arm.PEAK_OUTPUT_FORWARD_CONE_HIGH_POS),
                new WaitCommand(0.9).andThen(new ArmOut(Constants.TelescopingArm.HIGH_CONE_POS))
            ),
            new WheelsIn(),
            new WaitCommand(0.2),
            new ArmIn(),
            new GoHomeStageHome(),
            trajectoryWithEvents
        );
    }
}
