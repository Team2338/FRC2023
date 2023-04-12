package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.AutoLock;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;
import team.gif.robot.commands.autos.lib.UntilBotIsFallingReverseToward;
import team.gif.robot.commands.autos.lib.UntilBotIsLevel;
import team.gif.robot.commands.collector.CollectorEject;
import team.gif.robot.commands.combo.GoHomeStageHome;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class EngageFromFarSideReverse extends SequentialCommandGroup {
    /**
     * Coming from the far side (center, opponent) <br>
     * Starting from an angled position <br>
     * Method will crawl up until engaged
     *
     */
    public EngageFromFarSideReverse(){
        addCommands(
            new PrintCommand("Beginning EFFS"),
            new ParallelDeadlineGroup(
                new UntilBotIsFallingReverseToward().withTimeout(8),               // monitor gyro until level
                new AutoDrive(-Constants.AutoConstants.DRIVE_SLOW)
            ),
            new PrintCommand("Starting Kickback from far side"),
            new AutoDrive(Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.2), // give the bot a little push back to stop momentum
            //new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.00001)),
            new AutoLock(),
            new WaitCommand(0.2),
            new PrintCommand("Starting UBIL"),
            new UntilBotIsLevel().withTimeout(2),
            new PrintCommand("Bot is level - lock wheels"),
            new AutoLock(),
            new PrintCommand("Wheels locked")
//            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001))
        );
    }
}
