// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class UntilBotIsFallingReverseToward extends CommandBase {

    boolean falling = false;
    double targetAngle = -12.0;

    public UntilBotIsFallingReverseToward() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("UntilBotIsFallingReverseToward starting");
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // > -X is falling from opponent side facing away
        double pitch = Robot.pigeon.getPitch();

        if( pitch < -13) // need to get to -13 first, then can check for a smaller angle
            falling = true;

        System.out.println("UBIFRT falling: " + falling + " pitch: " + String.format("%.2f",pitch));

        if (falling && pitch > targetAngle) {
            System.out.println("UBIFRT Hit target angle of " + targetAngle);
            return true;
        } else {
            return false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        System.out.println("UBIFRT end");
    }
}
