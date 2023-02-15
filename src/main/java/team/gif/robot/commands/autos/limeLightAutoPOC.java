package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import team.gif.robot.subsystems.drivers.Limelight;

import team.gif.robot.Robot;

public class limeLightAutoPOC extends CommandBase {


    public limeLightAutoPOC() {
        addRequirements((Subsystem) Robot.limelight);
    }

    public void initialize() {

    }

    public void execute() {
        if (Robot.limelight.hasTarget()) {
            double xDirection = -Robot.limelight.getXOffset();
            Robot.swervetrain.drive(xDirection,0,0,true);
        }
    }
    public void end() {

    }

    public boolean isFinished() {
        return false;
    }
}
