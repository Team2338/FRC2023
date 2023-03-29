package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.autos.lib.AutoArmConeHigh;
import team.gif.robot.commands.collector.WheelsIn;
import team.gif.robot.commands.collector.WheelsOut;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;
import team.gif.robot.commands.telescopingArm.ArmOut;


public class PlaceConeHigh extends SequentialCommandGroup {

    /**
     * Raises elevator, extends arm <br>
     * Drops cone <br>
     * Retracts Arm <br>
     * <br>
     * Calling function must send elevator and arm home <br>
     * This allows for elevator and arm to retract while moving <br>
     * <br>
     */
    public PlaceConeHigh() {
        addCommands(
            new ParallelCommandGroup(
                new WheelsOut(),
                new SetArmPosition(Constants.Arm.STAGE_POS).withTimeout(5)
            ),
            new ParallelCommandGroup(
                new SetElevatorPosition(Constants.Elevator.PLACE_CONE_HIGH_POS).withTimeout(5),
                new AutoArmConeHigh().withTimeout(5),
                new WaitCommand(0.2).andThen(new ArmOut(Constants.TelescopingArm.HIGH_CONE_POS).withTimeout(5))
            ),
            new WheelsIn(),
            new WaitCommand(0.2),
            new ArmIn().withTimeout(5)
        );
    }
}
