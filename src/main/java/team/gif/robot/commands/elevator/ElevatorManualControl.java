package team.gif.robot.commands.elevator;

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

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

//-        System.out.println("elevator: " + Robot.elevator.getPosition());

        double speed = -Robot.oi.aux.getRightY();

        if (speed > -0.05 && speed < 0.05) {
            speed = 0;
        }

        // Allows user to run past 0 setpoint if pressing the right stick
        if (Robot.oi.aux.getRightStickButton()) {
            Robot.elevator.enableLowerSoftLimit(false);
        } else {
            Robot.elevator.enableLowerSoftLimit(true);
        }

        // run the elevator either up or down
        Robot.elevator.move(speed);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.elevator.move(0);
        Robot.elevator.enableLowerSoftLimit(true);
    }
}
