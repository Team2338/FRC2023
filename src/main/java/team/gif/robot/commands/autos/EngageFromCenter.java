package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;
import team.gif.robot.commands.autos.lib.UntilBotIsLevel;
import team.gif.robot.commands.combo.GoHomeStageHome;

public class EngageFromCenter extends SequentialCommandGroup {
    /**
     * Coming from the far side (center, opponent) <br>
     * Starting from an angled position <br>
     * Method will crawl up until engaged
     *
     */
    public EngageFromCenter(){
        addCommands(
            new ParallelDeadlineGroup(
                new UntilBotIsFalling(),               // monitor gyro until level
                new AutoDrive(Constants.AutoConstants.DRIVE_SLOW),
                new GoHomeStageHome()
            ),
            new AutoDrive(-Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.35), // give the bot a little push back to stop momentum
            new WaitCommand(0.5),
            new UntilBotIsLevel(),
            new UntilBotIsLevel(),
            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001))
        );
    }
}
