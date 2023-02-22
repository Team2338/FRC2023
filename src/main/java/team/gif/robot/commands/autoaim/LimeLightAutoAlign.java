package team.gif.robot.commands.autoaim;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.drivers.Limelight;

public class LimeLightAutoAlign extends CommandBase {

    private final double rTolerence = 2.0; // degrees
    private double rOffset;
    private final double yTolerence = 1.0; // degrees
    private double yOffset;

    public LimeLightAutoAlign() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.limelight.setLEDMode(Limelight.LED_ON); // turn on - just in case they were turned off somehow
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double velocity = 0;
        double rotation = 0;

        rOffset = -Robot.pigeon.get180Heading();
        rotation = (Math.abs(rOffset) < rTolerence) ? 0 : ((rOffset > 0) ? -0.8 : 0.8);

        if (Robot.limelight.hasTarget()) {
            yOffset = -Robot.limelight.getXOffset();
            velocity = (Math.abs(yOffset) < yTolerence) ? 0 : ((yOffset > 0) ? -0.4 : 0.4);
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,velocity,rotation);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if (Math.abs(yOffset) < yTolerence && Math.abs(rOffset) < rTolerence)
            return true;
        else
            return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0.0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }
}