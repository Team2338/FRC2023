package team.gif.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class ArmPIDControl extends CommandBase {

    private int counter=0;

    public ArmPIDControl() {
        super();
        addRequirements(Robot.arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        System.out.println("ARM PID CONTROL");
        Robot.arm.PIDMove();
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
