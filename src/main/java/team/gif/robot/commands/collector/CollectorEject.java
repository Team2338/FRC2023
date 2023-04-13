package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorEject extends CommandBase {
    private double ejectPercent;
    private final boolean getValueFromUI;

    public CollectorEject() {
        super();
        addRequirements(Robot.collector);
        ejectPercent = Constants.Collector.COLLECTOR_RUN_EJECT;
        getValueFromUI = false;
    }

    public CollectorEject(double ejectPercent) {
        super();
        addRequirements(Robot.collector);
        this.ejectPercent = ejectPercent;
        getValueFromUI = false;
    }

    public CollectorEject(boolean getValueFromUI) {
        super();
        addRequirements(Robot.collector);
        this.ejectPercent = 0;
        this.getValueFromUI = getValueFromUI;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (getValueFromUI)
            ejectPercent = Robot.uiSmartDashboard.COLLECTOR_EJECT_SPEED;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.collector.setSpeedPercentCollector(-ejectPercent);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Ejecting with " + ejectPercent);
        Robot.collector.setSpeedPercentCollector(0);
    }
}
