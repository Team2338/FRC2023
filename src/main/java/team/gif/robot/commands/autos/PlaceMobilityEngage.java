package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.UntilBotIsAngled;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;

public class PlaceMobilityEngage extends SequentialCommandGroup {
    public PlaceMobilityEngage() {
        addCommands(
//                new SetArmPosition(Constants.Arm.STAGE_POS),
//                new ParallelCommandGroup(
//                        new SetArmPosition(Constants.Arm.PLACE_CUBE_HIGH_POS),
//                        new SetElevatorPosition(Constants.Elevator.PLACE_CUBE_HIGH_POS)
//                ),
//                new CollectorEject().withTimeout(0.4),
//                new GoHomeStageHome(),
                new AutoDrive(-0.8).withTimeout(1),
                new AutoDrive(-0.8).withTimeout(3),
                new ParallelDeadlineGroup(
                        new UntilBotIsFalling(),               // monitor gyro until level
                        new AutoDrive(-0.8)

                ),
                new AutoDrive(-0.8).withTimeout(1),
                new AutoDrive(0.8).withTimeout(1),
                new ParallelDeadlineGroup(
                        new UntilBotIsAngled(),               // monitor gyro until level
                        new AutoDrive(0.8)
                ),
                new AutoDrive(0.8).withTimeout(1.2),
                new ParallelDeadlineGroup(
                        new UntilBotIsAngled(),               // monitor gyro until level
                        new AutoDrive(-0.8)
                ),
                new WaitCommand(0.5),               // given pigeon time to settle
                new ParallelDeadlineGroup(
                        new UntilBotIsFalling(),               // monitor gyro until level
                        new AutoDrive(0.8)
                ),
                new AutoDrive(0.4).withTimeout(.2)

//                new WaitCommand(.25),               // given pigeon time to settle
//                new ParallelDeadlineGroup(
//                        new UntilBotIsFalling(),               // monitor gyro until level
//                        new ForwardSlow()                      // drove forward slowly
//                ),
//                new ForwardFast().withTimeout(1.5)
        );
    }
}
