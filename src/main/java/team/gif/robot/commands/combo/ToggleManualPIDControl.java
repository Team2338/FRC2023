package team.gif.robot.commands.combo;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.ArmManualControl;
import team.gif.robot.commands.elevator.ElevatorManualControl;

public class ToggleManualPIDControl extends CommandBase {

    public ToggleManualPIDControl() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        new ArmManualControl().schedule();
        new ElevatorManualControl().schedule();
        Robot.arm.armManualFlag = true;
        Robot.elevator.elevatorManualFlag = true;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // By setting these to false, they will interrupt the manual control commands
        Robot.arm.armManualFlag = false;
        Robot.elevator.elevatorManualFlag = false;
    }
}
