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
import team.gif.robot.Robot;
import team.gif.robot.commands.autos.lib.CheckForGP;
import team.gif.robot.commands.autos.lib.SetInitialHeading;
import team.gif.robot.commands.collector.CollectorCollect;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.telescopingArm.ArmIn;

import java.util.HashMap;

public class ThreeGamePieceLeft extends SequentialCommandGroup {

    public ThreeGamePieceLeft() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("3GP Left", 1.6, 3.0); // 1.8 1.2
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("goHomeCollect", new ParallelCommandGroup(
                new ArmIn(),
                new AutoFloorCollectPos(),
                new WaitCommand(1).andThen( new CollectorCollect().until(Robot.arm.armGamePieceSensor::get).withTimeout(3.0))));

        eventMap.put("goHighCubeRear", new ParallelCommandGroup(
                new ArmIn(), // arm in just in case gravity pulled it out
                new AutoCubeHighRearPos()));

        eventMap.put("placeCubeHigh", new CollectorEject(0.80).withTimeout(0.25));

        eventMap.put("goCollectGP2", new ParallelCommandGroup(
                new AutoFloorCollectPos(),
                new ArmIn(),
                new WaitCommand(1).andThen( new CollectorCollect().until(Robot.arm.armGamePieceSensor::get).withTimeout(3.0))));

        eventMap.put("goMidCubeRear", new ParallelCommandGroup(
                new ArmIn(), // arm in just in case gravity pulled it out
                new AutoCubeMidRearPos()));

        eventMap.put("checkForGP", new CheckForGP());

        eventMap.put("placeCubeMid", new CollectorEject(.4).withTimeout(0.15));

        eventMap.put("goStage", new ParallelCommandGroup(
                new ArmIn(), // arm in just in case gravity pulled it out
                new AutoStagePos()));

        FollowPathWithEvents trajectoryWithEvents = new FollowPathWithEvents(
            RobotTrajectory.getInstance().baseSwerveCommand(trajectory, RobotTrajectory.getInstance().lowSpeedPIDConfig),
            trajectory.getMarkers(),
            eventMap
        );

        addCommands(
            new PrintCommand("Auto: 3 GP Left"),
//                new WheelsOut(),

            new SetInitialHeading(trajectory),
            new AutoConeMidRear(),
            new WheelsIn(),
            new WaitCommand(0.15),
            trajectoryWithEvents
//            new EngageFromFarSide()
        );
    }
}
