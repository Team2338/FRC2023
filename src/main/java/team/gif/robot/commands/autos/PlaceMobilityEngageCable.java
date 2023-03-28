package team.gif.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.lib.RobotTrajectory;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.autos.lib.AutoArmConeHigh;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.SetInitialHeading;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;
import team.gif.robot.commands.autos.lib.UntilBotIsLevel;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.collector.WheelsOut;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmOut;

import java.util.HashMap;

public class PlaceMobilityEngageCable extends SequentialCommandGroup {

    public PlaceMobilityEngageCable() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("PlaceMobEngage Cable", 2.0, 1.5); // 1.8 1.2
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
            new ParallelCommandGroup(
                new WheelsOut(),
                new SetArmPosition(Constants.Arm.STAGE_POS)
            ),
            new ParallelCommandGroup(
                new SetElevatorPosition(Constants.Elevator.PLACE_CONE_HIGH_POS),
                new AutoArmConeHigh(),
                //new SetArmPosition(Constants.Arm.PLACE_CONE_HIGH_POS),
                new WaitCommand(0.2).andThen(new ArmOut(Constants.TelescopingArm.HIGH_CONE_POS))
            ),
            new WheelsIn(),
            new WaitCommand(0.2),
            new ArmIn(),
            trajectoryWithEvents,
            new EngageFromCenter()

//            new ParallelDeadlineGroup(
//                    new UntilBotIsFalling(),               // monitor gyro until level
//                    new AutoDrive(Constants.AutoConstants.DRIVE_SLOW),
//                    new GoHomeStageHome()
//            ),
//            new AutoDrive(-Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.35), // give the bot a little push back to stop momentum
//            new WaitCommand(0.5),
//            new UntilBotIsLevel(),
//            new UntilBotIsLevel(),
//            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001))
        );
    }
}
