package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import team.gif.robot.Constants.Limelight;

import team.gif.robot.Robot;

public class limeLightAutoPOC extends CommandBase {


    public limeLightAutoPOC() {
        addRequirements((Subsystem) Robot.limelight);
    }

    public void initialize() {}

    public void execute() {
        if (Robot.limelight.hasTarget()) {
            double xOffset = -Robot.limelight.getXOffset();

            if (xOffset < Limelight.xOffsetOffRange && xOffset > Limelight.xOffsetOffRange) {
                xOffset = 0;
                return;
            }

            double output = Limelight.staticFrictionCoefficient + (Limelight.xOffsetMax/Math.abs(xOffset) * Limelight.kP);
            Robot.swervetrain.drive(0,output,0,true);
        }
    }
    public void end() {}

    public boolean isFinished() {
        return false;
    }
}
