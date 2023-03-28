package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.collector.WheelsOut;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmOut;

public class PlaceConeHighMobility extends SequentialCommandGroup {
    public PlaceConeHighMobility() {
        addCommands(
            new WheelsOut(),
            new SetArmPosition(Constants.Arm.STAGE_POS),
            new ParallelCommandGroup(
                new SetElevatorPosition(Constants.Elevator.PLACE_CONE_HIGH_POS),
                new SetArmPosition(Constants.Arm.PLACE_CONE_HIGH_POS, Constants.Arm.PEAK_OUTPUT_FORWARD_CONE_HIGH_POS),
                new WaitCommand(0.9).andThen(new ArmOut(Constants.TelescopingArm.HIGH_CONE_POS))
            ),
            new WheelsIn(),
            new WaitCommand(0.2),
            new ArmIn(),
            new GoHomeStageHome()
        );
    }
}
