package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.UntilBotIsAngled;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;
import team.gif.robot.commands.combo.GoHomeStageHome;

public class CenterMobilityEngage extends SequentialCommandGroup {
    /**
     * Drives up and over charging station. Then returns to charging station to Engage
     */
    public CenterMobilityEngage() {
        addCommands(
            new PrintCommand("Auto: Center Mobility Engage"),
            new ParallelCommandGroup(
                new GoHomeStageHome(),
                new WaitCommand(0.10).andThen(new AutoDrive(-0.7).withTimeout(3.2) )
            ),
            new ParallelDeadlineGroup(
                new UntilBotIsFalling().withTimeout(8),               // monitor gyro until falling
                new AutoDrive(-0.7)
            ),
            new AutoDrive(-0.5).withTimeout(0.55), //was 0.75->0.55 // finish driving over station
            new WaitCommand(0.5),                  // give charging station time to level and stop teetering
            // begin to drive back to alliance station
            new ParallelDeadlineGroup(
                new UntilBotIsAngled().withTimeout(5),               // monitor gyro until climbing
                new AutoDrive(0.7)
            ),
            new AutoDrive(0.7).withTimeout(0.9), // drive a little more up the ramp
            new EngageFromFarSide()
//                new ParallelDeadlineGroup(
//                        new UntilBotIsFalling(),               // monitor gyro until level
//                        new AutoDrive(Constants.AutoConstants.DRIVE_SLOW)
//                ),
//                new AutoDrive(-Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.35), // give the bot a little push back to stop momentum
//                new WaitCommand(0.5),
//                new UntilBotIsLevel(),
//                new UntilBotIsLevel(),
//                new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001))
        );
    }
}
