package team.gif.robot.commands.drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class ResetHeading extends CommandBase {
    private double heading = 0;

    public ResetHeading() {}

    public ResetHeading(double heading) {
        this.heading = heading;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.pigeon.resetPigeonPosition(heading);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
