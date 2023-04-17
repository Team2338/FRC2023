// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.arm.ArmPIDControl;

public class AutoArmConeHigh extends CommandBase {

    public AutoArmConeHigh(){
        super();
        addRequirements(Robot.arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.arm.move(Constants.Arm.AUTO_OUTPUT_CONE_HIGH_POS);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.arm.getPosition() > (Constants.Arm.PLACE_CONE_HIGH_POS - (3.0 * Constants.Arm.TICKS_PER_DEGREE));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        Robot.arm.setTargetPosition(Constants.Arm.PLACE_CONE_HIGH_POS - (3.0 * Constants.Arm.TICKS_PER_DEGREE));
        Robot.arm.configPeakOutputForward(Constants.Arm.PEAK_OUTPUT_FORWARD);
        Robot.arm.configI(Constants.Arm.I_GT_45);
        Robot.arm.resetI();
        Robot.arm.PIDMove();
    }
}
