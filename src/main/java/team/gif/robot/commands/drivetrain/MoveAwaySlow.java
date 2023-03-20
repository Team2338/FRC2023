package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class MoveAwaySlow extends CommandBase {

    public MoveAwaySlow() {
        addRequirements(Robot.swervetrain); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // move at 0.5 m/s (or 0.2 m/s if right stick is pressed)
        if( Robot.oi.driver.getHID().getRightStickButton())
            Robot.swervetrain.drive(0.2, 0, 0.0);
        else
            Robot.swervetrain.drive(0.5, 0, 0.0);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swervetrain.drive(0.0, 0.0, 0.0);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }
}
