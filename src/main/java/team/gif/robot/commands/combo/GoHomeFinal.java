package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class GoHomeFinal extends SequentialCommandGroup {
    public GoHomeFinal() {
        addCommands(

            new SetArmPosition( Constants.Arm.ARM_80),
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.MOVE_FROM_HOME_PRE_POS),
                new SetElevatorPosition(Constants.Elevator. MAX_HOME_SAFE_POS)
            ),
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.HOME_POS),
                new SetElevatorPosition(Constants.Elevator.HOME_POS)
            )
//            new GoLocation()
        );
    }
}
