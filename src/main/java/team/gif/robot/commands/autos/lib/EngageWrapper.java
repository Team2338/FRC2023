package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Robot;

public class EngageWrapper extends SequentialCommandGroup {
    public EngageWrapper(){
        addCommands(
            new UntilBotIsLevel().withTimeout(3.0),
            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001))
        );
    }
}
