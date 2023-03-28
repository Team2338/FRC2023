package team.gif.robot.commands.autoaim;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class LimeLightAutoAlign extends CommandBase {

    private final double rTolerence = 1.0; // degrees
    private double rOffset;
    private final double yTolerence = 1.0; // degrees
    private double yOffset;
    private int passCount;
    private int executeCount;

    public LimeLightAutoAlign() {
        addRequirements(Robot.swervetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.limelightLow.setLEDOn();
        passCount = 0;
        executeCount=0;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0.01,0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double velocitySpeed = 0;
        double rotationSpeed = 0;

        if( executeCount++ < 3) // give time for LED to turn on
            return;

//        rOffset = -(Robot.pigeon.get360Heading() - 180); // bot faces 180 degrees when placing game pieces
//        rotationSpeed = (Math.abs(rOffset) < rTolerence) ? 0 : ((rOffset > 0) ? -0.4 : 0.4);

        if (Robot.limelightLow.hasTarget()) {
            yOffset = Robot.limelightLow.getXOffset();
            velocitySpeed = (Math.abs(yOffset) < yTolerence) ? 0 : ((yOffset > 0) ? -0.2 : 0.2);
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,velocitySpeed,rotationSpeed);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if( Robot.oi.driver.getHID().getXButtonPressed()) // need a kill switch
            return true;

        if (Math.abs(yOffset) < yTolerence && Math.abs(rOffset) < rTolerence) {
            if (++passCount < 11) {
                return false; // need to check that the bot didn't overrun
            } else {
                return true; // aligned! Fire away!!
            }
        } else {
            return false; // not in tolerance, return false
        }
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.limelightLow.setLEDOff();
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.01, 0, 0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
    }
}
