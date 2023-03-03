package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.commands.autos.lib.UntilBotIsFalling;

public class EngageTest extends SequentialCommandGroup {
    public EngageTest(){
        addCommands(
            new AutoDrive(-Constants.AutoConstants.DRIVE_FAST).withTimeout(SmartDashboard.getNumber("AutoTime", 2.3)), // drive (time and speed based) until we are angled
            new WaitCommand(.25),               // given pigeon time to settle
            new ParallelDeadlineGroup(
                new UntilBotIsFalling(),               // monitor gyro until level
                new AutoDrive(-Constants.AutoConstants.DRIVE_SLOW)
            ),
            new AutoDrive(Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.2)       // give the bot a little push back to stop momentum
        );
    }
}
