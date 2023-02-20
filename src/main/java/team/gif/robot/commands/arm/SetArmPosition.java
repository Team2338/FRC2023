package team.gif.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

/**
 * Sets the arm to an angle
 */
public class SetArmPosition extends CommandBase {

    private final double targetPosition;

    public SetArmPosition(double targetPos) {
        super();
        addRequirements(Robot.arm);

        // do not allow code to set a point higher or lower than max/min
        if (targetPos > Constants.Arm.MAX_POS) { targetPos = Constants.Arm.MAX_POS; }
        if (targetPos < Constants.Arm.MIN_POS) { targetPos = Constants.Arm.MIN_POS; }

        targetPosition = targetPos;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.arm.setTargetPosition(targetPosition);

        if (Robot.arm.PIDError() > 0){
            Robot.arm.configF(Constants.Arm.FF);
            Robot.arm.configP(Constants.Arm.P);
        } else {
            Robot.arm.configF(Constants.Arm.REV_FF);
            Robot.arm.configP(Constants.Arm.REV_P);
        }
        Robot.arm.PIDMove();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.arm.isFinished();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
