package team.gif.robot.commands.telescopingArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ArmLoadFromDouble extends CommandBase {

    public ArmLoadFromDouble() {
        super();
        addRequirements(Robot.telescopingArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double sign = Robot.telescopingArm.getPosition() < Constants.TelescopingArm.HIGH_COLLECT_POS ? 1.0 : -1.0;

        if ( Math.abs(Robot.telescopingArm.getPosition() - Constants.TelescopingArm.HIGH_COLLECT_POS) < 8.0) {
            Robot.telescopingArm.setMotorSpeed(sign * Constants.TelescopingArm.LOW_VELOCITY);
        } else {
            Robot.telescopingArm.setMotorSpeed(sign * Constants.TelescopingArm.HIGH_VELOCITY);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if (Math.abs(Robot.telescopingArm.getPosition() - Constants.TelescopingArm.HIGH_COLLECT_POS) < 1.0) {
            return true;
        } else
            return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.telescopingArm.setMotorSpeed(0);
    }
}
