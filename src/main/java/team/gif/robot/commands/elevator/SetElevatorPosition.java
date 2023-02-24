package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class SetElevatorPosition extends CommandBase {

    private final double desiredPosition;

    public SetElevatorPosition(double targetPosition) {
        super();
        addRequirements(Robot.elevator);

        // do not allow code to set a point higher or lower than max/min
        if (targetPosition > Constants.Elevator.MAX_POS) { targetPosition = Constants.Elevator.MAX_POS; }
        if (targetPosition < Constants.Elevator.MIN_POS) { targetPosition = Constants.Elevator.MIN_POS; }

        desiredPosition = targetPosition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.elevator.setElevatorTargetPos(desiredPosition);

        if (desiredPosition > Robot.elevator.getPosition()) {
            Robot.elevator.setCruiseVelocity(Constants.Elevator.MAX_VELOCITY);
            Robot.elevator.configF(Constants.Elevator.F);
            Robot.elevator.setMotionMagic(desiredPosition, Constants.Elevator.GRAV_FEED_FORWARD);
        } else {
            Robot.elevator.setCruiseVelocity(Constants.Elevator.REV_MAX_VELOCITY);
            Robot.elevator.configF(Constants.Elevator.REV_F);
            Robot.elevator.setMotionMagic(desiredPosition, Constants.Elevator.REV_GRAV_FEED_FORWARD);
        }
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
//        if( Robot.arm.getPositionDegrees() < Constants.Arm.MOVE_FROM_HOME_POS && Robot.elevator.getPosition() > Constants.Elevator.MAX_HOME_SAFE_POS)
//            return false; // TODO: prevent arm from crashing into top rail
        return Robot.elevator.isFinished();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
