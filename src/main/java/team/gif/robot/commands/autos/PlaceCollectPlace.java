package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PlaceCollectPlace extends SequentialCommandGroup {
    public PlaceCollectPlace(){
        addCommands(
                new AutoDrive(1.2).withTimeout(1.5),
                new AutoDrive(0.3).withTimeout(1.5),
                new AutoDrive(-0.3).withTimeout(1.5),
                new AutoDrive(-1.2).withTimeout(1.5)
        );
    }
}
