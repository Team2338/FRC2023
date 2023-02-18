package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.SetArmPosition;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.telescopingArm.ArmIn;

public class GoFloor extends CommandBase {
    public GoFloor() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.elevator.getPosition() < Constants.Elevator.MAX_HOME_SAFE_POS) {
            new GoFloorFromHome().schedule();
        }
        else {
            new ArmIn().schedule();
            new SetArmPosition(Constants.Arm.LOAD_FROM_GROUND_POS).schedule();
            new SetElevatorPosition(Constants.Elevator.LOAD_FROM_GROUND_POS).schedule();
        }
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
