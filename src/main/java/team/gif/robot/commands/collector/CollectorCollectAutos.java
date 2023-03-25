package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorCollectAutos extends CommandBase {

    public CollectorCollectAutos() {
        super();
        addRequirements(Robot.collector);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.collector.setSpeedPercentCollector(Constants.Collector.COLLECTOR_RUN_COLLECT);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.arm.armGamePieceSensor.get();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.collector.setSpeedPercentCollector(0);
    }
}
