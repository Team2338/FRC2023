package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.UntilBotIsAngled;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;
import team.gif.robot.commands.autos.lib.UntilBotIsLevel;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class PlaceCubeHighMobilityEngage extends SequentialCommandGroup {
    public PlaceCubeHighMobilityEngage() {
        addCommands(
                new SetArmPosition(Constants.Arm.STAGE_POS),
                new ParallelCommandGroup(
                        new SetArmPosition(Constants.Arm.PLACE_CUBE_HIGH_POS, Constants.Arm.PEAK_OUTPUT_FORWARD_CUBE_HIGH_POS),
                        new SetElevatorPosition(Constants.Elevator.PLACE_CUBE_HIGH_POS)
                ),
                new CollectorEject().withTimeout(0.15),
                new CenterMobilityEngage()
        );
    }
}
