package team.gif.robot.commands.telescopingArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ArmIn extends CommandBase {
    private double location;

    public ArmIn() {
        super();
        addRequirements(Robot.telescopingArm);
        this.location = Constants.TelescopingArm.MIN_POS; // arm safe position
    }

    public ArmIn(double location) {
        this();
        this.location = location;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        System.out.println("ARM IN");
        if (Robot.telescopingArm.getPosition() > Constants.TelescopingArm.SLOW_POS) { // use the mid position to start slowing down
            Robot.telescopingArm.setMotorSpeed(-Constants.TelescopingArm.HIGH_VELOCITY);
        } else if(Robot.telescopingArm.getPosition() < Constants.TelescopingArm.MIN_POS) {
            Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.LOW_VELOCITY);
        } else {
            Robot.telescopingArm.setMotorSpeed(-Constants.TelescopingArm.LOW_VELOCITY);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if (Robot.telescopingArm.getPosition() < location) {
            System.out.println("arm done");
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
