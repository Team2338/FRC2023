package team.gif.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class ConeLEDs extends CommandBase {

    public ConeLEDs() {
        super();
        addRequirements(Robot.led);
    }

    // Called when the command is initially scheduled.
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.led.setLEDColor(100,100,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    public void end(boolean interrupted) {}
}
