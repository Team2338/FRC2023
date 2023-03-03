package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;

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
                new ForwardFast().withTimeout(3),
                new ParallelDeadlineGroup(
                        new UntilBotIsFalling(),               // monitor gyro until level
                        new ForwardFast()
                ),
                new ForwardFast().withTimeout(1),
                new ReverseFast().withTimeout(1.2),
                new ParallelDeadlineGroup(
                        new UntilBotIsAngled(),               // monitor gyro until level
                        new ReverseFast()
                ),
                new ReverseFast().withTimeout(1.2),
                new ParallelDeadlineGroup(
                        new UntilBotIsAngled(),               // monitor gyro until level
                        new ReverseFast()
                ),
                new WaitCommand(0.5),               // given pigeon time to settle
                new ParallelDeadlineGroup(
                        new UntilBotIsFalling(),               // monitor gyro until level
                        new ReverseSlow()                      // drove forward slowly
                ),
                new ForwardSlow().withTimeout(.2)       // give the bot a little push back to stop momentum

//                new WaitCommand(.25),               // given pigeon time to settle
//                new ParallelDeadlineGroup(
//                        new UntilBotIsFalling(),               // monitor gyro until level
//                        new ForwardSlow()                      // drove forward slowly
//                ),
//                new ForwardFast().withTimeout(1.5)
        );
    }
}
