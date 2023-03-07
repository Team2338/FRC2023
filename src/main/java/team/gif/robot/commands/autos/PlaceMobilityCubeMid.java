package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.collector.WheelsOut;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;

public class PlaceMobilityCubeMid extends SequentialCommandGroup {
    public PlaceMobilityCubeMid() {
        addCommands(
            new WheelsIn(),
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.PLACE_CUBE_MID_POS),
                new SetElevatorPosition(Constants.Elevator.PLACE_CUBE_MID_POS)
            ),
            new CollectorEject().withTimeout(0.5),
            new GoHomeStageHome()
        );
    }
}
