// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class UntilBotIsFallingFarSide extends CommandBase {

    boolean falling = false;

    /**
     * Bot facing us, on far side, driving toward us, from far side to the charging station
     * Results in positive angle when initially climbing
     */
    public UntilBotIsFallingFarSide() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("UntilBotIsFallingFarSide starting");

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
        // greater than because bot goes from 0 to 15
        if (pitch > Robot.uiSmartDashboard.FFS_CROSSOVER_ANGLE)
            falling = true;

        System.out.println("UBIFFS (C " + String.format("%.1f",Robot.uiSmartDashboard.FFS_CROSSOVER_ANGLE) + " F " + String.format("%.1f",Robot.uiSmartDashboard.FFS_FALLING_ANGLE) +  ") falling: " + falling + " pitch: " + String.format("%.2f",pitch));

        // less than because bot is falling, going from 15 to 0
        if (falling && pitch < Robot.uiSmartDashboard.FFS_FALLING_ANGLE) {
            System.out.println("UBIFFS Hit target angle of " + Robot.uiSmartDashboard.FFS_FALLING_ANGLE);
            return true;
        } else {
            return false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){System.out.println("UBIFFS end");}
}
