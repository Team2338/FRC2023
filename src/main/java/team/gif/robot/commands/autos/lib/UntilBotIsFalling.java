// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class UntilBotIsFalling extends CommandBase {

    public UntilBotIsFalling() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
//        System.out.println("checking pitch " + Robot.pigeon.getPitch());
        // > -X is coming from alliance station and falling to become level
        // < X is falling from opponent side
        return Robot.pigeon.getPitch() > -12.0 && Robot.pigeon.getPitch() < 10.0; // was -14.0 // MW -10 and 12
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}
}
