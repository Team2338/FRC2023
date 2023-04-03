package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

public class DriveAndEngage extends SequentialCommandGroup {
    public DriveAndEngage(){
        addCommands(
//            new AutoDrive(-Constants.AutoConstants.DRIVE_FAST).withTimeout(SmartDashboard.getNumber("AutoTime", 2.3)), // drive (time and speed based) until we are angled
            new PrintCommand("Auto: Drive and Engage"),
            new DriveToChargingStation(),
            new AutoDrive(-Constants.AutoConstants.HOLD_AT_ANGLE).withTimeout(.25),               // give pigeon time to settle
            new ParallelDeadlineGroup(
                new UntilBotIsFalling().withTimeout(8),               // monitor gyro until level
                new AutoDrive(-Constants.AutoConstants.DRIVE_SLOW),
                new GoHomeStageHome()
            ),
            new PrintCommand("Reached -10 degrees. Starting kickback"),
            new AutoDrive(Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.25), // was // give the bot a little push back to stop momentum
            new WaitCommand(0.3),

            new UntilBotIsLevel().withTimeout(5),
            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001))
        );
    }
}
