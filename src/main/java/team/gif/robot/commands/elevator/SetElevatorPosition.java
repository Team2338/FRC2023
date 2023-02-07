package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.Elevator;

public class SetElevatorPosition extends CommandBase {

    private final double position;
    private int loopCounter;

    public SetElevatorPosition(int targetPosition) {
        super();
        // do not allow code to set a point higher or lower than max/min
        if (targetPosition > Constants.Elevator.MAX_POS) { targetPosition = Constants.Elevator.MAX_POS; }
        if (targetPosition < Constants.Elevator.MIN_POS) { targetPosition = Constants.Elevator.MIN_POS; }

        position = targetPosition;

        addRequirements(Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        loopCounter = 0;

        if (position > Robot.elevator.getPosition()) {
            Robot.elevator.setCruiseVelocity(Constants.Elevator.MAX_VELOCITY);
            Robot.elevator.configF(Constants.Elevator.F);
            Robot.elevator.setMotionMagic(position, Constants.Elevator.GRAV_FEED_FORWARD);
        } else {
            Robot.elevator.setCruiseVelocity(Constants.Elevator.REV_MAX_VELOCITY);
            Robot.elevator.configF(Constants.Elevator.REV_F);
            Robot.elevator.setMotionMagic(position, Constants.Elevator.REV_GRAV_FEED_FORWARD);
        }
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if( ++loopCounter > 20 ) // waiting for 20 cycles (400ms) to let error catch up
            return Robot.elevator.isFinished();
        else
            return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
