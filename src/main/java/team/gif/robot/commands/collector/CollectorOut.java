package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class CollectorOut extends CommandBase {
    public CollectorOut() {
        super();
        addRequirements(Robot.collectorPneumatics);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.collectorPneumatics.collectorOut();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
