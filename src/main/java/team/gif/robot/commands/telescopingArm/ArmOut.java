package team.gif.robot.commands.telescopingArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ArmOut extends CommandBase {

    double location;

    public ArmOut(double location) {
        super();
        addRequirements(Robot.telescopingArm);
        this.location = location;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.telescopingArm.getPosition() < Constants.TelescopingArm.HIGH_POS * .80) {
            Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.HIGH_VELOCITY);
        } else {
            Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.LOW_VELOCITY);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if (Robot.telescopingArm.getPosition() > location) {
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
