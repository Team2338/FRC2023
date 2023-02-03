package team.gif.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class MoveUp extends CommandBase {

    private int counter=0;

    public MoveUp() {
        super();
        addRequirements(Robot.arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.arm.PIDMove(1750);
        System.out.println(Robot.arm.getTicks() + "  " + Robot.arm.PIDError());
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Math.abs(Robot.arm.PIDError()) < 10;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
