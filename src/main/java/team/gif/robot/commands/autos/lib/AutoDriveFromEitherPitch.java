// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AutoDriveFromEitherPitch extends CommandBase {

    private double speed;
    private double initialPitch;

    public AutoDriveFromEitherPitch(double speed) {
        addRequirements(Robot.swervetrain);
        this.speed = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        initialPitch = Robot.pigeon.getPitch();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (initialPitch > 4.0) {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-speed, 0.0, 0.0);
            SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
            Robot.swervetrain.setModuleStates(moduleStates);
        } else if (initialPitch < -4.0) {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0.0, 0.0);
            SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
            Robot.swervetrain.setModuleStates(moduleStates);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Auto drive interrupted. Give drivetrain kick-back");
        if (initialPitch > 4.0)
            new AutoDrive(Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.35).schedule();
        else if (initialPitch < -4.0)
            new AutoDrive(-Constants.AutoConstants.DRIVE_SUPER_SLOW).withTimeout(.35).schedule();
    }
}
