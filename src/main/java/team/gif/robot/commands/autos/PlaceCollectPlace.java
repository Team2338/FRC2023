package team.gif.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.lib.path.RobotTrajectory;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.CollectorCollect;
import team.gif.robot.commands.collector.CollectorCollectAutos;
import team.gif.robot.commands.combo.GoFloorFromHome;

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
//
//        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Return GP One Barrier", 3, 6);
//        HashMap<String, Command> eventMap2 = new HashMap<>();
//        eventMap.put("Score", new PlaceCubeHighMobility());
//
//        FollowPathWithEvents trajectoryWithEventsReturn = new FollowPathWithEvents(
//                RobotTrajectory.getInstance().baseSwerveCommand(trajectory1),
//                trajectory1.getMarkers(),
//                eventMap2
//        );

        addCommands(
            trajectoryWithEvents
        );
    }
}
