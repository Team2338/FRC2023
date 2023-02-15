package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmTest;
import team.gif.robot.commands.arm.SetArmMid;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class GoFloorFromHome extends SequentialCommandGroup {
    public GoFloorFromHome() {
        addCommands(
//                new SetArmTest(),
//                new ParallelCommandGroup(
//                        new SetArmMid(),
//                        new SetElevatorPosition(Constants.Elevator.LOAD_FROM_SINGLE_SUBSTATION_POS)
//                ),
//                new SetArmTest()
        );
    }
}
