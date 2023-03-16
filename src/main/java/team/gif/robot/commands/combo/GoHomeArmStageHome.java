package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;

public class GoHomeArmStageHome extends SequentialCommandGroup {
    public GoHomeArmStageHome() {
        addCommands(
            new ArmIn(),
            new SetArmPosition(Constants.Arm.ARM_80),
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.STAGE_POS),
                new SetElevatorPosition(Constants.Elevator.STAGE_POS)
            )
//            new ParallelCommandGroup(
//                new SetArmPosition(Constants.Arm.HOME_POS),
//                new SetElevatorPosition(Constants.Elevator.HOME_POS)
//            )
        );
    }
}
