package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;
import team.gif.robot.OI;

public class UntilCollect extends CommandBase {
    public UntilCollect() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {return Robot.arm.armGamePieceSensor.get();}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}
}
