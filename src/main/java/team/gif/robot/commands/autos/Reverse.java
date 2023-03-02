// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class Reverse extends CommandBase {

    public Reverse() {
        addRequirements(Robot.swervetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.3, 0.0, 0.0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
  }
}
