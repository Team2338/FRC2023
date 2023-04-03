package team.gif.robot.commands.autoaim;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.CollectorCollect;

public class LimeLightAutoCollect extends CommandBase {
    private final double rTolerence = 1.0; // degrees
    private double rOffset;
    private final double yTolerence = 5.0; // degrees
    private double xOffset;
    private Command collectorCollectSchedule = new CollectorCollect();

    int count = 0;

    public LimeLightAutoCollect() {addRequirements(Robot.swervetrain);}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        collectorCollectSchedule.schedule();
        count = 0;
        Robot.ledSubsystem.clearLEDGamePieceColor();
        Robot.ledSubsystem.setLEDAutoCollectActive();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double rotationSpeed = 0;
        if (count < 20) {
            if (Robot.limelightHigh.hasTarget()) {
                xOffset = Robot.limelightHigh.getXOffset();
                rotationSpeed = (Math.abs(xOffset) < yTolerence) ? 0 : ((xOffset > 0) ? -0.4 : 0.4);
                if (rotationSpeed == 0) {
                    count++;
                } else {
                    count = 0;
                }
            }

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,rotationSpeed); //turns towards xoffset and drives forward
            SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
            Robot.swervetrain.setModuleStates(moduleStates);
        }

        if (Robot.elevator.getPosition() > (Constants.Elevator.LOAD_FROM_DOUBLE_SUBSTATION_POS - 3 * Constants.Elevator.EL_TICKS_PER_INCH) &&
            Robot.arm.getPosition() > (Constants.Arm.LOAD_FROM_DOUBLE_SUBSTATION_POS - 5 * Constants.Arm.TICKS_PER_DEGREE) &&
            Robot.arm.getPosition() < (Constants.Arm.LOAD_FROM_DOUBLE_SUBSTATION_POS + 3 * Constants.Arm.TICKS_PER_DEGREE)) {


            Robot.ledSubsystem.clearLEDGamePieceColor();

            if (count == 20 && Robot.telescopingArm.getPosition() < Constants.TelescopingArm.MAX_POS) {
                Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.HIGH_VELOCITY);
            } else {
                Robot.telescopingArm.setMotorSpeed(0);
            }
        } else {
            Robot.ledSubsystem.setLEDAutoAlignError();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if( Robot.oi.driver.getHID().getXButtonPressed()) // need a kill switch
            return true;
        return Robot.arm.armGamePieceSensor.get();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.ledSubsystem.clearLEDGamePieceColor();
        Robot.ledSubsystem.setLEDHPColor(0, 0, 0);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0.0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
        collectorCollectSchedule.cancel();
    }
}
