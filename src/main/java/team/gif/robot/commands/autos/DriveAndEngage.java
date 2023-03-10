package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;

public class DriveAndEngage extends SequentialCommandGroup {
    public DriveAndEngage(){
        addCommands(
//            new AutoDrive(-Constants.AutoConstants.DRIVE_FAST).withTimeout(SmartDashboard.getNumber("AutoTime", 2.3)), // drive (time and speed based) until we are angled
            new DriveToChargingStation(),
            new WaitCommand(.25),               // give pigeon time to settle
            new ParallelDeadlineGroup(
                new UntilBotIsFalling(),               // monitor gyro until level
                new AutoDrive(-Constants.AutoConstants.DRIVE_SLOW)
            ),
            new AutoDrive(Constants.AutoConstants.DRIVE_SLOW).withTimeout(.4) // give the bot a little push back to stop momentum
        );
    }
}
