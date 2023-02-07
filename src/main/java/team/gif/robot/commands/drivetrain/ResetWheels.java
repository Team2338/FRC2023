package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class ResetWheels extends CommandBase {
    public ResetWheels() {
        addRequirements(Robot.swervetrain);
    }

    @Override
    public void execute() {
        Robot.swervetrain.fL.resetWheel();
        Robot.swervetrain.fR.resetWheel();
        Robot.swervetrain.rL.resetWheel();
        Robot.swervetrain.rR.resetWheel();
    }
}
