package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.UntilBotIsAngled;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class PlaceCubeHighMobilityEngage extends SequentialCommandGroup {
    public PlaceCubeHighMobilityEngage() {
        addCommands(
                new SetArmPosition(Constants.Arm.STAGE_POS),
                new ParallelCommandGroup(
                        new SetArmPosition(Constants.Arm.PLACE_CUBE_HIGH_POS),
                        new SetElevatorPosition(Constants.Elevator.PLACE_CUBE_HIGH_POS)
                ),
                new CollectorEject().withTimeout(0.15),
                new ParallelCommandGroup(
                    new GoHomeStageHome(),
                    new WaitCommand(0.10).andThen(new AutoDrive(-0.7).withTimeout(3.2) )
                ),

                new ParallelDeadlineGroup(
                        new UntilBotIsFalling(),               // monitor gyro until level
                        new AutoDrive(-0.7)
                ),
                new AutoDrive(-0.5).withTimeout(0.75),
                new WaitCommand(0.5),
                // begin to drive back to alliance station
                new ParallelDeadlineGroup(
                        new UntilBotIsAngled(),               // monitor gyro until level
                        new AutoDrive(0.7)
                ),
                new AutoDrive(0.7).withTimeout(1.0),
                new ParallelDeadlineGroup(
                        new UntilBotIsAngled(),               // monitor gyro until level
                        new AutoDrive(0.7)
                ),
                new WaitCommand(0.25),               // given pigeon time to settle
                new ParallelDeadlineGroup(
                        new UntilBotIsFalling(),               // monitor gyro until level
                        new AutoDrive(0.3)
                ),
                new AutoDrive(-Constants.AutoConstants.DRIVE_SLOW).withTimeout(.4)
        );
    }
}
