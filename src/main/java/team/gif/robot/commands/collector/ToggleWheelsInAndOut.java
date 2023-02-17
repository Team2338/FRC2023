package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class ToggleWheelsInAndOut extends CommandBase {
    public ToggleWheelsInAndOut() {
        super();
        addRequirements(Robot.collectorPneumatics);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Wheels In");
        Robot.collectorPneumatics.pneumaticsIn();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Wheels Out");
        Robot.collectorPneumatics.pneumaticsOut();
    }
}
