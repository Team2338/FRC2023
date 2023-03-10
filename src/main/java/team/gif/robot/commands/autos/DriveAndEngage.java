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
            new AutoDrive(Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.35), // give the bot a little push back to stop momentum
            new WaitCommand(0.5),

            new UntilBotIsLevel().withTimeout(0.5),
            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.000001))
        );
    }
}
