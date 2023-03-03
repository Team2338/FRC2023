package team.gif.lib.actions;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import team.gif.robot.Robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

@SuppressWarnings("MemberName")
public class SwerveTrajectoryAction implements Action {
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final SwerveDriveKinematics kinematics;
    private final HolonomicDriveController controller;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final Supplier<Rotation2d> desiredRotation;
    private final Supplier<Boolean> wantsVisionAlign;

    @SuppressWarnings("ParameterName")
    public SwerveTrajectoryAction(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Supplier<Rotation2d> desiredRotation,
            Supplier<Boolean> wantsVisionAlign,
            Consumer<SwerveModuleState[]> outputModuleStates
    ) {
        this.trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveTrajectoryAction");
        this.pose = requireNonNullParam(pose, "pose", "SwerveTrajectoryAction");
        this.kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveTrajectoryAction");
        this.controller =
            new HolonomicDriveController(
                    requireNonNullParam(xController, "xController", "SwerveTrajectoryAction"),
                    requireNonNullParam(yController, "yController", "SwerveTrajectoryAction"),
                    requireNonNullParam(thetaController, "thetaController", "SwerveTrajectoryAction")
            );
        this.outputModuleStates = requireNonNullParam(outputModuleStates, "output", "SwerveTrajectoryAction");
        this.desiredRotation = requireNonNullParam(desiredRotation, "desiredRotation", "SwerveTrajectoryAction");
        this.wantsVisionAlign =
                requireNonNullParam(wantsVisionAlign, "wantsVisionAlign", "SwerveTrajectoryAction");
    }

    @SuppressWarnings("ParameterName")
    public SwerveTrajectoryAction(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates
    ) {
        this(
            trajectory,
            pose,
            kinematics,
            xController,
            yController,
            thetaController,
            () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
            () -> false,
            outputModuleStates
        );
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        double currTime = timer.get();
        Trajectory.State desiredState = trajectory.sample(currTime);
        Rotation2d desiredRotation = new Rotation2d();

        if(wantsVisionAlign.get()) {
        }
    }

    @Override
    public void done() {
        timer.stop();
        outputModuleStates.accept(kinematics.toSwerveModuleStates((ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)))));
    }

    public Pose2d getInitialPose() {
        return trajectory.getInitialPose();
    }
}
