package team.gif.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class DriveArcade extends CommandBase {
    public DriveArcade() {
        super();
        addRequirements(Robot.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rot = Robot.oi.driver.getRightX() * .95;
        double currSpd = Robot.oi.driver.getLeftY();
        Robot.drivetrain.driveArcade(rot,currSpd);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
