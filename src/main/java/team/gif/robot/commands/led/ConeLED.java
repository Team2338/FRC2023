package team.gif.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class ConeLED extends CommandBase {

    public ConeLED() {
        super();
    }

    // Called when the command is initially scheduled.
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.ledSubsystem.setLEDHPColor(255,255,0);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    public void end(boolean interrupted) {
        Robot.ledSubsystem.setLEDHPColor(0,0,0);
    }
}
