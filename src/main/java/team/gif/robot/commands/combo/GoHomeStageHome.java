package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class GoHomeStageHome extends SequentialCommandGroup {
    public GoHomeStageHome() {
        addCommands(
            new SetArmPosition(Constants.Arm.ARM_50), // safe pos
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.HOME_POS),
                new SetElevatorPosition(Constants.Elevator.HOME_POS)
            )
        );
    }
}
