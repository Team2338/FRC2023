package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.combo.GoHomeFinal;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class PlaceMobilityCone extends SequentialCommandGroup {
    public PlaceMobilityCone() {
        addCommands(
            new ParallelCommandGroup(
                new SetArmPosition(Constants.Arm.PLACE_CONE_MID_POS),
                new SetElevatorPosition(Constants.Elevator.PLACE_CONE_MID_POS)
            ),
            new WheelsIn(),
            new GoHomeFinal()
        );
    }
}
