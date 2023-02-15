package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmMid;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.arm.SetArmTest;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class GoLocationFromHome extends SequentialCommandGroup {
    public GoLocationFromHome() {
        addCommands(
            new SetArmPosition(Constants.Arm.MOVE_FROM_HOME_POS),
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.MOVE_FROM_HOME_PRE_POS),
                new SetElevatorPosition(Constants.Elevator.MOVE_FROM_HOME_PRE_POS)
            )
        );
    }
}
