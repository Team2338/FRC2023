package team.gif.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class ElevatorManualControl extends CommandBase {

    public ElevatorManualControl() {
        super();
        addRequirements(Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double percent = -Robot.oi.aux2.getRightY();

        if (percent > -0.05 && percent < 0.05) {
            percent = 0.05; // apply minimum FeedForward to keep the elevator from falling (0.10 is max before elevator begins to move)
        }

        Robot.elevator.move(percent);

        // Allows user to run past 0 setpoint if pressing the right stick
/*        if (Robot.oi.aux.getRightStickButton()) {
            Robot.elevator.enableLowerSoftLimit(false);
        } else {
            Robot.elevator.enableLowerSoftLimit(true);
        }
*/
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return !Robot.elevator.elevatorManualFlag;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.elevator.move(0.05);
        Robot.elevator.setElevatorTargetPos(Robot.elevator.getPosition());
        Robot.elevator.enableLowerSoftLimit(true);
    }
}
