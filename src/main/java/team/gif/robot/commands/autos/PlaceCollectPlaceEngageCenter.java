package team.gif.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.lib.RobotTrajectory;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.autos.lib.AutoLock;
import team.gif.robot.commands.autos.lib.CheckForGP;
import team.gif.robot.commands.autos.lib.SetInitialHeading;
import team.gif.robot.commands.collector.CollectorCollect;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmOut;

import java.util.HashMap;

public class PlaceCollectPlaceEngageCenter extends SequentialCommandGroup {

    public PlaceCollectPlaceEngageCenter() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("2 GP Center", 1.1, 1.5); // 1.8 1.2
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("goStage", new ParallelCommandGroup(
                new ArmIn(), // arm in just in case gravity pulled it out
                new AutoStagePos()));

        eventMap.put("goHomeCollect", new ParallelCommandGroup(
                new AutoFloorCollectPos(),
                new WaitCommand(1).andThen( new CollectorCollect().until(Robot.arm.armGamePieceSensor::get).withTimeout(3.0))));

        eventMap.put("goStageBack", new ParallelCommandGroup(
                new ArmIn(), // arm in just in case gravity pulled it out
                new AutoStagePos()));

        FollowPathWithEvents trajectoryWithEvents = new FollowPathWithEvents(
            RobotTrajectory.getInstance().baseSwerveCommand(trajectory),
            trajectory.getMarkers(),
            eventMap
        );

        addCommands(
            new PrintCommand("Auto: 2 GP Center Engage"),
            new SetInitialHeading(trajectory),
            new AutoConeMidRear(),
            new WheelsIn(),
            new WaitCommand(0.15),
            new CollectEngageCenter(),
            new PrintCommand("Checking for GP"),
            new CheckForGP(), // cancels rest of auto if there is no GP
            new PrintCommand("Arm Out"),
            new ArmOut(Constants.TelescopingArm.HIGH_CONE_POS),
            new CollectorEject(true).withTimeout(0.75)
        );
    }
}
