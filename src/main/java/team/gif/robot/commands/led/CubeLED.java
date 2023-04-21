
package team.gif.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class CubeLED extends CommandBase {

    public CubeLED() {
        super();
    }

    // Called when the command is initially scheduled.
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    public void execute() {
        Robot.ledSubsystem.setLEDHPColor(255,0,255);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    public void end(boolean interrupted) {
        Robot.ledSubsystem.setLEDHPColor(0,0,0);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}