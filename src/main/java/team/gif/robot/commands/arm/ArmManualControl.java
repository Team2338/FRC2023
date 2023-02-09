package team.gif.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class ArmManualControl extends CommandBase {

    private boolean holdNeedFirstPID;
    private double holdPIDPos;

    public ArmManualControl() {
        super();
        addRequirements(Robot.arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        holdNeedFirstPID = false;
        holdPIDPos = Robot.arm.getPosition();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double percent = Robot.oi.aux.getLeftY();

        if (percent > -0.05 && percent < 0.05) {
            percent = 0;
            if( holdNeedFirstPID ) {
                holdPIDPos = Robot.arm.getPosition();
                holdNeedFirstPID = false;
            }
            Robot.arm.armMotor.set(ControlMode.Position, holdPIDPos);
        } else {
            holdNeedFirstPID = true;
            Robot.arm.move(percent);
        }

        // Allows user to run past 0 setpoint if pressing the right stick
//        if (Robot.oi.aux.getRightStickButton()) {
//            Robot.climber.enableLowerSoftLimit(false);
//        } else {
//            Robot.climber.enableLowerSoftLimit(true);
//        }

        // run the elevator either up or down
//        Robot.arm.move(percent);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return !Robot.arm.armManualFlag;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.arm.move(0);
        Robot.arm.setArmTargetPos(Robot.arm.getPosition());
    }
}
