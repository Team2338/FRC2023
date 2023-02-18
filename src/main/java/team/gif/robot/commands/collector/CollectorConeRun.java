package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorConeRun extends CommandBase {
    public CollectorConeRun() {
        super();
        addRequirements(Robot.collector);
        addRequirements(Robot.collectorPneumatics);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.collectorPneumatics.pneumaticsIn();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.collector.setSpeedPercentCollector(Constants.Collector.COLLECTOR_RUN);
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
