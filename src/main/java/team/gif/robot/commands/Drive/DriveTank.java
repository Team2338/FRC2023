package team.gif.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class DriveTank extends CommandBase {
    public DriveTank() {
        super();
        addRequirements(Robot.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currL = 0; //-Robot.oi.driver.getLeftY();
        double currR = 0; // -Robot.oi.driver.getRightY();
        Robot.drivetrain.driveTank(currL,-currR);
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
