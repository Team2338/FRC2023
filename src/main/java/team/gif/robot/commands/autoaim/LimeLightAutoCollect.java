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
//        if (Robot.collectorWheels.getWheelState()) {
//            Robot.limelightHigh.setPipeline(1);
//        } else {
//            Robot.limelightHigh.setPipeline(0);
//        }
        collectorCollectSchedule.schedule();
        count = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double rotationSpeed = 0;
        if (count < 20) {
            if (Robot.limelightHigh.hasTarget()) {
                xOffset = -Robot.limelightHigh.getXOffset();
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
        if (count == 20 && Robot.telescopingArm.getPosition() < Constants.TelescopingArm.MAX_POS) {
            Robot.telescopingArm.setMotorSpeed(Constants.TelescopingArm.HIGH_VELOCITY);
        } else {
            Robot.telescopingArm.setMotorSpeed(0);
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
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0.0);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.swervetrain.setModuleStates(moduleStates);
        collectorCollectSchedule.cancel();
    }
}
