package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class GoHomeConditional extends CommandBase {
    /**
     * Going Home is now automatic when the robot detects a game piece.
     * <br>
     * <b>Exceptions:</b><br>
     * <i>Single Substation:</i> This is drive team controlled in order to back out of the station
     * before moving the arm/elevator. Otherwise, the robot crashes into the loading station
     */
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

        if( !Robot.runningAutonomousMode)
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
