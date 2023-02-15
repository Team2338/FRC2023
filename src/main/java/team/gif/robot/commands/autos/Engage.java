// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

/** An example command that uses an example subsystem. */
public class Engage extends CommandBase {

    int counter = 0;

  public Engage() {
    addRequirements(Robot.swervetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1.4, 0.0, 0.0);
      SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
      Robot.swervetrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
      SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
      Robot.swervetrain.setModuleStates(moduleStates);
      System.out.println("ENDING" + ++counter);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    if(Math.abs(Robot.swervetrain.fL.getDriveMotor().getEncoder().getPosition()) < 36.5) {
//        System.out.println("RUNNING " + ++counter);
//        return false;
//    } else {
//        System.out.println("EXITING");
//        return true;
//    }
      return true;
  }
}
