// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class LevelBot extends CommandBase {

    public LevelBot() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // continuously monitor the gyro angle
        double pitch = Robot.pigeon.getPitch();

        double [] accel = Robot.pigeon.getAccelAngles();

        System.out.println("accel :" + accel[0] + " " + accel[1] + " " + accel[2]);

        if (Math.abs(pitch) < 10.0)
            Robot.swervetrain.drive(0,0,0.0001);
        else if (pitch > 0)
            Robot.swervetrain.drive(Constants.AutoConstants.DRIVE_SLOW, 0, 0);
        else
            Robot.swervetrain.drive(-Constants.AutoConstants.DRIVE_SLOW, 0, 0);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}
}
