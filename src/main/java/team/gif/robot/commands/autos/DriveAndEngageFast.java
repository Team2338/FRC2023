package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.AutoDriveFromEitherPitch;
import team.gif.robot.commands.autos.lib.LevelBot;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;
import team.gif.robot.commands.autos.lib.UntilBotIsLevel;
import team.gif.robot.commands.combo.GoHomeStageHome;

public class DriveAndEngageFast extends SequentialCommandGroup {
    public DriveAndEngageFast(){
        addCommands(
//            new AutoDrive(-Constants.AutoConstants.DRIVE_FAST).withTimeout(SmartDashboard.getNumber("AutoTime", 2.3)), // drive (time and speed based) until we are angled
            new DriveToChargingStation(),
            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001)),
            new WaitCommand(.25),               // give pigeon time to settle
            new ParallelDeadlineGroup(
                new UntilBotIsFalling(),               // monitor gyro until level
                new AutoDriveFromEitherPitch(Constants.AutoConstants.DRIVE_SLOW), // this assumes the bot did not go over the charging station
                new GoHomeStageHome()
            ),
            new WaitCommand(0.5),

            new UntilBotIsLevel(),
            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001)),

            new WaitCommand(0.3),
            new UntilBotIsLevel(),
            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001)),

            new WaitCommand(0.3),
            new UntilBotIsLevel(),
            new InstantCommand(()-> Robot.swervetrain.drive(0,0,0.0001))
        );
    }
}
