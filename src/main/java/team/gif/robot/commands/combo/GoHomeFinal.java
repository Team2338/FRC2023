package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.subsystems.Arm;

import java.nio.file.Watchable;

public class GoHomeFinal extends SequentialCommandGroup {
    public GoHomeFinal() {
        addCommands(
                new ArmIn(),
            new SetArmPosition(Constants.Arm.ARM_80),
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.MOVE_FROM_HOME_PRE_POS),
                new SetElevatorPosition(Constants.Elevator.MAX_HOME_SAFE_POS)
            ),
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.HOME_POS),
                new SetElevatorPosition(Constants.Elevator.HOME_POS)
            )
        );
    }
}
