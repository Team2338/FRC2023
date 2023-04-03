// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class UntilBotIsLevel extends CommandBase {

    private int posAngle = 0;
    public UntilBotIsLevel() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        posAngle = 0;
        if (Robot.pigeon.getPitch() > 5.0) { //overshot
            posAngle = 2;
        } else if (Robot.pigeon.getPitch() < -5.0) {
            posAngle = 1;
        }
        System.out.println("UBL posAngle :" + posAngle);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if(posAngle == 2) {
            Robot.swervetrain.drive(-0.2, 0, 0); // drive toward alliance wall fRel
        } else if(posAngle == 1) {
            Robot.swervetrain.drive(0.2, 0, 0); // drive away from alliance fRel
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        System.out.println("UBL pitch:" + Robot.pigeon.getPitch());
        return Math.abs(Robot.pigeon.getPitch()) < 4.0;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        System.out.println("Finishing Until Bot Is Level");
    }
}
