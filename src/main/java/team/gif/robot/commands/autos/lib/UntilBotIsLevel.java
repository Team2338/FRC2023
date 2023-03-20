// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class UntilBotIsLevel extends CommandBase {

    private double initialPitch;

    public UntilBotIsLevel() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Checking for Level Bot (UntilBotIsLevel)");
        initialPitch = Robot.pigeon.getPitch();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        System.out.println("Checking ...");
        if (initialPitch > 4.0) {
            System.out.println("         ... bot is not level");
            Robot.swervetrain.drive(-Constants.AutoConstants.DRIVE_SUPER_SLOW, 0, 0);
        } else if (initialPitch < -4.0) {
            System.out.println("         ... bot is not level");
            Robot.swervetrain.drive(Constants.AutoConstants.DRIVE_SUPER_SLOW, 0, 0);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Math.abs(Robot.pigeon.getPitch()) < 2.0;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        System.out.println("Bot is level. Exiting");
    }
}
