package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.collector.WheelsOut;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class PlaceCubeHighMobility extends SequentialCommandGroup {
    public PlaceCubeHighMobility() {
        addCommands(
            new WheelsIn(),
            new SetArmPosition(Constants.Arm.STAGE_POS),
            new ParallelCommandGroup(
                    new SetArmPosition(Constants.Arm.PLACE_CUBE_HIGH_POS),
                    new SetElevatorPosition(Constants.Elevator.PLACE_CUBE_HIGH_POS)
            ),
            new CollectorEject().withTimeout(0.4),
            new GoHomeStageHome(),
//            new ForwardFast().withTimeout(3.0)
            new AutoDrive(-Constants.AutoConstants.DRIVE_FAST).withTimeout(3.0)
        );
    }
}
