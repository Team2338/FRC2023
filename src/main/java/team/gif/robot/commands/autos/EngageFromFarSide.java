package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;
import team.gif.robot.commands.autos.lib.UntilBotIsLevel;
import team.gif.robot.commands.combo.GoHomeStageHome;

public class EngageFromFarSide extends SequentialCommandGroup {
    /**
     * Coming from the far side (center, opponent) <br>
     * Starting from an angled position <br>
     * Method will crawl up until engaged
     *
     */
    public EngageFromFarSide(){
        addCommands(
            new ParallelDeadlineGroup(
                new UntilBotIsFalling().withTimeout(8),               // monitor gyro until level
                new AutoDrive(Constants.AutoConstants.DRIVE_SLOW),
                new GoHomeStageHome()
            ),
            new PrintCommand("Starting Kickback from far side"),
            new AutoDrive(-Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.15), // give the bot a little push back to stop momentum
            new WaitCommand(0.3),
            new UntilBotIsLevel().withTimeout(5),
            new UntilBotIsLevel().withTimeout(5),
            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001))
        );
    }
}
