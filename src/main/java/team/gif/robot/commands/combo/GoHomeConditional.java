package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class GoHomeConditional extends CommandBase {
    public GoHomeConditional() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(Math.abs(Robot.elevator.getPosition() - Constants.Elevator.LOAD_FROM_SINGLE_SUBSTATION_POS) < (1 * Constants.Elevator.EL_TICKS_PER_INCH)) {
            if(Math.abs(Robot.arm.getPosition() - Constants.Arm.LOAD_FROM_SINGLE_SUBSTATION_POS) < (3 * Constants.Arm.TICKS_PER_DEGREE)) {
                return;
            }
        }

        new GoHome().schedule();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
