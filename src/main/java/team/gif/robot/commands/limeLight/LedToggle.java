package team.gif.robot.commands.limeLight;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.subsystems.drivers.Limelight;
import team.gif.robot.Robot;

public class LedToggle extends CommandBase {
    public LedToggle(){
        super();
    }

    // Called when the command is initially scheduled.
    public void initialize() {
        Robot.limelight.setLEDMode(Limelight.LED_ON);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {return false;}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.limelight.setLEDMode(Limelight.LED_OFF);
    }
}
