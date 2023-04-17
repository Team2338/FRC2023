// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class UntilBotIsFalling extends CommandBase {

    boolean falling = false;

    /**
     * Bot facing us, in between Alliance Station and Charging Station, driving away (in reverse), from AS to the CS
     * Results in negative angle when initially climbing
     */
    public UntilBotIsFalling() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("UntilBotIsFalling starting");

        falling = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        double pitch = Robot.pigeon.getPitch();

        // need to be climbing, then can check for a smaller angle
        // less than because bot goes from 0 to -15
        if (pitch < Robot.uiSmartDashboard.CROSSOVER_ANGLE)
            falling = true;

        System.out.println("UBIF (C " + String.format("%.1f",Robot.uiSmartDashboard.CROSSOVER_ANGLE) + " F " + String.format("%.1f",Robot.uiSmartDashboard.FALLING_ANGLE) + ") falling: " + falling + " pitch: " + String.format("%.2f",pitch));

        // greater than because bot is falling, going from -15 to 0
        if (falling && pitch > Robot.uiSmartDashboard.FALLING_ANGLE) {
            System.out.println("UBIF Hit target angle of " + Robot.uiSmartDashboard.FALLING_ANGLE);
            return true;
        } else {
            return false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){System.out.println("UBIF end");}
}
