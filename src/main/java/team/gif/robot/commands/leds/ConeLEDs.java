package team.gif.robot.commands.leds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;




public class ConeLEDs extends CommandBase {
    public ConeLEDs() {
        super();
        addRequirements(Robot.led);
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        Robot.led.setLEDOrange();
    }

    // Called once the command ends or is interrupted.

    public void end(boolean interrupted) {
    }
}
    // Returns true when the command should end.
