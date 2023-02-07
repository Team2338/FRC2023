package frc.robot.commands.LEDcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class DefaultLED extends CommandBase {

    public DefaultLED() {
        addRequirements(Robot.led);
    }

    public void initialize() {


    }

    // Called every time the scheduler runs while the command is scheduled.

    public void execute() {
        Robot.led.setLEDNull();
    }

    // Called once the command ends or is interrupted.

    public void end(boolean interrupted) {}

    // Returns true when the command should end.

    public boolean isFinished() {
        return false;
    }
}

