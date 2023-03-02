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

public class PlaceCubeHighEngage extends SequentialCommandGroup {
    public PlaceCubeHighEngage(){
        addCommands(
            new SetArmPosition(Constants.Arm.STAGE_POS),
            new ParallelCommandGroup(
                    new SetArmPosition(Constants.Arm.PLACE_CUBE_HIGH_POS),
                    new SetElevatorPosition(Constants.Elevator.PLACE_CUBE_HIGH_POS)
            ),
            new CollectorEject().withTimeout(0.4),
            new GoHomeStageHome(),

            new ForwardFast().withTimeout(2.3), // drive (time and speed based) until we are angled
            new WaitCommand(.25),               // given pigeon time to settle
            new ParallelDeadlineGroup(
                new UntilBotIsFalling(),               // monitor gyro until level
                new ForwardSlow()                      // drove forward slowly
            ),
            new Reverse().withTimeout(.2)       // give the bot a little push back to stop momentum
        );
    }
}
