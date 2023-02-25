package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class GoFloorFromHome extends SequentialCommandGroup {
    public GoFloorFromHome() {
        addCommands(
            new GoLocationFromHomeArmStage(),
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.LOAD_FROM_GROUND_POS),
                new SetElevatorPosition(Constants.Elevator.LOAD_FROM_GROUND_POS)
            )
        );
    }
}
