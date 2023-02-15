package team.gif.robot.commands.limeLight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import team.gif.robot.subsystems.drivers.Limelight;

import static team.gif.robot.Robot.limelight;


public class LedToggle extends CommandBase {
    public LedToggle(){
        addRequirements((Subsystem) limelight);
    }

    public void initialize() {

        limelight.setLEDMode(Limelight.LED_ON);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        limelight.setLEDMode(Limelight.LED_OFF);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
