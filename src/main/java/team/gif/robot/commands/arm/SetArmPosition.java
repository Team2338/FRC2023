package team.gif.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class SetArmPosition extends CommandBase {

    private final double targetPosition;

    public SetArmPosition(double targetPos) {
        super();
        addRequirements(Robot.arm);
        // do not allow code to set a point higher or lower than max/min
        if (targetPos > Constants.Arm.MAX_POS) { targetPos = Constants.Arm.MAX_POS; }
        if (targetPos < Constants.Arm.MIN_POS) { targetPos = Constants.Arm.MIN_POS; }

        Robot.arm.setTargetPosition(targetPos);
        targetPosition = targetPos;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize( ) {

        if (targetPosition > Robot.arm.getPosition()) {
            Robot.arm.setCruiseVelocity(Constants.Arm.MAX_VELOCITY);
            Robot.arm.configF(Constants.Arm.F);
            Robot.arm.setMotionMagic(targetPosition, Constants.Arm.GRAV_FEED_FORWARD);
        } else {
            System.out.println("raise arm");
            Robot.arm.setCruiseVelocity(Constants.Arm.REV_MAX_VELOCITY);
            Robot.arm.configF(Constants.Arm.REV_F);
            Robot.arm.setMotionMagic(targetPosition, Constants.Arm.REV_GRAV_FEED_FORWARD);
        }

        Robot.arm.setTargetPosition(targetPosition);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        System.out.println("START ARM MOTION MAGIC");
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.arm.isFinished();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.arm.setTargetPosition(Robot.arm.getPosition()); // update target based on current position
        System.out.println("END ARM MOTION MAGIC");
    }
}
