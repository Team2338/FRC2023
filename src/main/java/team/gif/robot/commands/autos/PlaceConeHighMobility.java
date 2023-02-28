package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmOut;

public class PlaceConeHighMobility extends SequentialCommandGroup {
    public PlaceConeHighMobility() {
        addCommands(
//            new SetArmPosition(30 * Constants.Arm.TICKS_PER_DEGREE + Constants.Arm.ZERO_OFFSET_TICKS),
//            new SetElevatorPosition(Constants.Elevator.MAX_HOME_SAFE_POS),
//            new ParallelCommandGroup(
//                new SetArmPosition(45 * Constants.Arm.TICKS_PER_DEGREE + Constants.Arm.ZERO_OFFSET_TICKS),
//                new SetElevatorPosition(30 * Constants.Elevator.EL_TICKS_PER_INCH - Constants.Elevator.ZERO_OFFSET_TICKS)
//            ),
            new ParallelCommandGroup(
                new SetElevatorPosition(Constants.Elevator.PLACE_CONE_HIGH_POS),
                new SetArmPosition(Constants.Arm.PLACE_CONE_HIGH_POS, Constants.Arm.VELOCITY_GO_HIGH_FROM_HOME),
                new WaitCommand(0.9).andThen(new ArmOut(Constants.TelescopingArm.HIGH_POS))
            ),
            new WheelsIn(),
            new WaitCommand(0.5),
            new ArmIn(),
            new GoHomeStageHome()
        );
    }
}
