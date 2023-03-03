package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.autos.lib.AutoDrive;

public class PlaceCubeHighEngage extends SequentialCommandGroup {
    public PlaceCubeHighEngage(){
        addCommands(
//            new SetArmPosition(Constants.Arm.STAGE_POS),
//            new ParallelCommandGroup(
//                    new SetArmPosition(Constants.Arm.PLACE_CUBE_HIGH_POS),
//                    new SetElevatorPosition(Constants.Elevator.PLACE_CUBE_HIGH_POS)
//            ),
//            new CollectorEject().withTimeout(0.4),
//            new GoHomeStageHome(),

            new AutoDrive(-Constants.AutoConstants.DRIVE_FAST).withTimeout(2.3) // drive (time and speed based) until we are angled
//-            new WaitCommand(.25),               // given pigeon time to settle
//-            new ParallelDeadlineGroup(
//-                new UntilBotIsFalling(),               // monitor gyro until level
//-                new AutoDrive(-Constants.AutoConstants.DRIVE_SLOW)
//-            )
//-            new AutoDrive(Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.2)       // give the bot a little push back to stop momentum
        );
    }
}
