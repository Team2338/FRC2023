package team.gif.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ArmManualControl extends CommandBase {

    public ArmManualControl() {
        super();
        addRequirements(Robot.arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = -Robot.oi.aux.getLeftY();

        if (speed > -0.05 && speed < 0.05) {
            speed = 0;
        }

        // Allows user to run past 0 setpoint if pressing the right stick
//        if (Robot.oi.aux.getRightStickButton()) {
//            Robot.climber.enableLowerSoftLimit(false);
//        } else {
//            Robot.climber.enableLowerSoftLimit(true);
//        }

        // run the elevator either up or down
        Robot.arm.move(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.arm.move(0);
//        Robot.climber.enableLowerSoftLimit(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
