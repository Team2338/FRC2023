// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class UntilBotIsFallingReverseToward extends CommandBase {

    boolean falling = false;

    /**
     * Bot facing away, on far side, driving toward us (in reverse), from far side to the charging station
     * Results in negative angle when initially climbing
     */
    public UntilBotIsFallingReverseToward() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("UntilBotIsFallingReverseToward starting");

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
        if( pitch < Robot.uiSmartDashboard.UBIFRT_CROSSOVER_ANGLE)
            falling = true;

        System.out.println("UBIFRT (C " + String.format("%.1f",Robot.uiSmartDashboard.UBIFRT_CROSSOVER_ANGLE) + " F " + String.format("%.1f",Robot.uiSmartDashboard.UBIFRT_FALLING_ANGLE) + ") falling: " + falling + " pitch: " + String.format("%.2f",pitch));

        // if we have overshot right from the start, exit
        if( pitch > 0 ) {
            System.out.println("UBIFRT Path Overshot! Exiting UBIFRT");
            return true;
        }

        // greater than because bot is falling, going from -15 to 0
        if (falling && pitch > Robot.uiSmartDashboard.UBIFRT_FALLING_ANGLE) {
            System.out.println("UBIFRT Hit target angle of " + Robot.uiSmartDashboard.UBIFRT_FALLING_ANGLE);
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
