package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class SetElevatorPosition extends CommandBase {

    private final double targetPosition;

    public SetElevatorPosition(int targetPos) {
        super();
        addRequirements(Robot.elevator);

        // do not allow code to set a point higher or lower than max/min
        if (targetPos > Constants.Elevator.MAX_POS) { targetPos = Constants.Elevator.MAX_POS; }
        if (targetPos < Constants.Elevator.MIN_POS) { targetPos = Constants.Elevator.MIN_POS; }

        Robot.elevator.setElevatorTargetPos(targetPos);
        targetPosition = targetPos;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        if (targetPosition > Robot.elevator.getPosition()) {
            Robot.elevator.setCruiseVelocity(Constants.Elevator.MAX_VELOCITY);
            Robot.elevator.configF(Constants.Elevator.F);
            Robot.elevator.setMotionMagic(targetPosition, Constants.Elevator.GRAV_FEED_FORWARD);
        } else {
            Robot.elevator.setCruiseVelocity(Constants.Elevator.REV_MAX_VELOCITY);
            Robot.elevator.configF(Constants.Elevator.REV_F);
            Robot.elevator.setMotionMagic(targetPosition, Constants.Elevator.REV_GRAV_FEED_FORWARD);
        }
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // this is currently always returning false, but keep the logc here in case
        // it does return true and we need to shift to PID control
//        return Robot.elevator.elevatorMotor.isMotionProfileFinished();
//        System.out.println("MOTION MAGIC");
        return Robot.elevator.isFinished();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
//        Robot.elevator.setElevatorTargetPos(Robot.elevator.getPosition());
    }
}
