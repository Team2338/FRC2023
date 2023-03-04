package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmOut;

public class GoHighFromHome extends SequentialCommandGroup {
    public GoHighFromHome() {
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorPosition(Constants.Elevator.PLACE_CONE_HIGH_POS),
                new SetArmPosition(Constants.Arm.PLACE_CONE_HIGH_POS),
                new WaitCommand(1).andThen(new ArmOut(Constants.TelescopingArm.HIGH_CONE_POS))
            )
        );
    }
}
