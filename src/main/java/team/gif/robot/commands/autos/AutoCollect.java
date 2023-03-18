package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.commands.autoaim.LimeLightAutoCollect;
import team.gif.robot.commands.autos.lib.UntilCollect;

public class AutoCollect extends SequentialCommandGroup {
    boolean HasCollected = false;

    public AutoCollect() {
        addCommands(
            new ParallelDeadlineGroup(
                    new UntilCollect(),
                    new LimeLightAutoCollect()
            )
        );
    }
}
