package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;

public class GoHomeArmStageHome extends SequentialCommandGroup {
    public GoHomeArmStageHome() {
        addCommands(
//            new ArmIn(),
//            new ParallelCommandGroup(
//                new SetArmPosition(Constants.Arm.STAGE_POS),
//                new SetElevatorPosition(Constants.Elevator.STAGE_POS)
//            )

            new ParallelCommandGroup(
                new ArmIn(),
                new WaitUntilCommand(Robot.telescopingArm::safePos).andThen(new SetArmPosition(Constants.Arm.STAGE_POS)),
                new WaitUntilCommand(Robot.telescopingArm::safePos).andThen(new SetElevatorPosition(Constants.Elevator.STAGE_POS))
            )
        );
    }
}
