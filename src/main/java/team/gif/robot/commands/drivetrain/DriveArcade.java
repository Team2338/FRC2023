package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class DriveArcade extends CommandBase {
    public DriveArcade() {
        super();
        addRequirements(Robot.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled.
    @Override
    public void execute() {
        double rot = 0; //Robot.oi.driver.getRightX() * .95;
        double currSpd = 0; //Robot.oi.driver.getLeftY();
        Robot.drivetrain.driveArcade(rot,currSpd);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

}
