package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.autos.lib.AutoDrive;
import team.gif.robot.Robot;


public class DriveToChargingStation extends SequentialCommandGroup {
    public DriveToChargingStation(){
        addCommands(
            new AutoDrive(-Constants.AutoConstants.DRIVE_FAST).withTimeout(SmartDashboard.getNumber("Auto Time", Constants.AutoConstants.DRIVE_TIME_DEFAULT))
        );
    }
}
