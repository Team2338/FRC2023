package team.gif.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class LEDSubsystemDefault extends CommandBase {

    public LEDSubsystemDefault() {
        super();
        addRequirements(Robot.ledSubsystem);
    }

    // Called when the command is initially scheduled.
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    public void execute() {
        Robot.ledSubsystem.setColors();
    }

    // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    public void end(boolean interrupted) {}

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
