package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmOut;

public class PlaceMobilityConeHigh extends SequentialCommandGroup {
    public PlaceMobilityConeHigh() {
        addCommands(
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.PLACE_CONE_HIGH_POS),
                new SetElevatorPosition(Constants.Elevator.PLACE_CONE_HIGH_POS),
                new ArmOut(Constants.TelescopingArm.HIGH_POS)
            ),
            new WheelsIn(),
            new ArmIn(),
            new GoHomeStageHome()
        );
    }
}
