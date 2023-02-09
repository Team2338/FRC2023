package team.gif.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class ElevatorManualControl extends CommandBase {

    private boolean holdNeedFirstPID;
    private double holdPIDPos;

    public ElevatorManualControl() {
        super();
        addRequirements(Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        holdNeedFirstPID = false;
        holdPIDPos = Robot.elevator.getPosition();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double percent = -Robot.oi.aux.getRightY();

        if (percent > -0.03 && percent < 0.03) {
            if( holdNeedFirstPID ) {
                holdPIDPos = Robot.elevator.getPosition();
                holdNeedFirstPID = false;
            }
            Robot.elevator.elevatorMotor.set(ControlMode.Position, holdPIDPos);
        } else {
            holdNeedFirstPID = true;
            Robot.elevator.move(percent);
        }

        // Allows user to run past 0 setpoint if pressing the right stick
        if (Robot.oi.aux.getRightStickButton()) {
            Robot.elevator.enableLowerSoftLimit(false);
        } else {
            Robot.elevator.enableLowerSoftLimit(true);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return !Robot.elevator.elevatorManualFlag;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.elevator.move(0);
        Robot.elevator.setElevatorTargetPos(Robot.elevator.getPosition());
        Robot.elevator.enableLowerSoftLimit(true);
    }
}
