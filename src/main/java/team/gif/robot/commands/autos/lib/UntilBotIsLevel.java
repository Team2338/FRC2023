// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class UntilBotIsLevel extends CommandBase {

    private int posAngle = 0;
    private boolean invert = false;
    private double sign = 1;

    public UntilBotIsLevel() {
        super();
        invert = false;
    }

    public UntilBotIsLevel(boolean invert) {
        super();
        this.invert = invert;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("UBIL (2nd chance): Starting");

        sign = invert ? -1 : 1;

        posAngle = 0;
        if (Robot.pigeon.getPitch() > Robot.uiSmartDashboard.THRESHOLD_ANGLE) { //overshot
            posAngle = 2;
        } else if (Robot.pigeon.getPitch() < -1 * Robot.uiSmartDashboard.THRESHOLD_ANGLE) {
            posAngle = 1;
        } else {
            System.out.println("Bot is already level. 2nd chance is not necessary.");
        }
        System.out.println("UBIL posAngle :" + posAngle);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        if(posAngle == 2) {
            Robot.swervetrain.drive(sign * -1 * 0.2, 0, 0); // drive toward alliance wall fRel
        } else if(posAngle == 1) {
            Robot.swervetrain.drive(sign * 0.2, 0, 0); // drive away from alliance fRel
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        System.out.println("UBIL pitch (" + posAngle + " " + String.format("%.1f",Robot.uiSmartDashboard.LEVEL_ANGLE) + "):" + String.format("%.2f",Robot.pigeon.getPitch()));
        return Math.abs(Robot.pigeon.getPitch()) < Robot.uiSmartDashboard.LEVEL_ANGLE;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        Robot.swervetrain.drive(0, 0, 0);
        System.out.println("UBIL Finish at: " + String.format("%.2f",Robot.pigeon.getPitch()) + " degrees");
    }
}
