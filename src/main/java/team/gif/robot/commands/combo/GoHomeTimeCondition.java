package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;
import team.gif.robot.commands.combo.GoHome;

public class GoHomeTimeCondition extends CommandBase {

    public GoHomeTimeCondition() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.collector.getTimer() > 2.0) {
            new GoHome().schedule();
        }
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
}
