package team.gif.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class SetArmHigh extends CommandBase {

    private int counter;

    public SetArmHigh() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.arm.setArmTargetPos(1600);
        Robot.arm.PIDMove();
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if ( ++counter > 15)  // waiting for 20 cycles (400ms) to let error catch up
            return Robot.arm.isFinished();
        else
            return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
