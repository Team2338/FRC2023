// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AutoLock extends CommandBase {

    int count = 0;

    public AutoLock() {
        addRequirements(Robot.swervetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        count=0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.swervetrain.drive2(0, 0, -.2);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        System.out.println("count " + count);
        return ++count > 2;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("drive at 0,0,0");
        Robot.swervetrain.drive2(0, 0, 0);
    }
}
