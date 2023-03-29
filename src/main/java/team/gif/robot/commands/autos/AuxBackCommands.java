// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;
import team.gif.robot.commands.combo.ToggleManualPIDControl;

public class AuxBackCommands extends CommandBase {

    public AuxBackCommands() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if( Robot.oi.dX.getAsBoolean())
            new PlaceCollectPlaceBarrier().schedule();
        else
            new ToggleManualPIDControl().schedule();
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
