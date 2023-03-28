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
import team.gif.robot.commands.autos.lib.AutoArmConeHigh;
import team.gif.robot.commands.autos.lib.SetInitialHeading;
import team.gif.robot.commands.collector.CollectorCollect;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.collector.WheelsOut;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmOut;

import java.util.HashMap;

public class PlaceCollectPlaceBarrier extends SequentialCommandGroup {

    public PlaceCollectPlaceBarrier() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("PlaceCollectPlace Barrier", 1.8, 1.2);
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("goHome", new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.STAGE_POS),
                new ArmIn(),
                new SetElevatorPosition(Constants.Elevator.STAGE_POS)));
        eventMap.put("armDown", new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.LOAD_FROM_GROUND_POS),
                new SetElevatorPosition(Constants.Elevator.LOAD_FROM_GROUND_POS),
                new WaitCommand(1).andThen( new CollectorCollect().until(Robot.arm.armGamePieceSensor::get).withTimeout(3.0))));
        eventMap.put("armUp", new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.STAGE_POS),
                new SetElevatorPosition(Constants.Elevator.STAGE_POS)));
        eventMap.put("armPlace", new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.PLACE_CUBE_HIGH_POS),
                new SetElevatorPosition(Constants.Elevator.PLACE_CUBE_HIGH_POS)));

        FollowPathWithEvents trajectoryWithEvents = new FollowPathWithEvents(
                RobotTrajectory.getInstance().baseSwerveCommand(trajectory),
                trajectory.getMarkers(),
                eventMap
        );

        addCommands(
            new SetInitialHeading(trajectory),
            new PlaceConeHigh(),
            trajectoryWithEvents,
            new CollectorEject().withTimeout(1.0)
        );
    }
}
