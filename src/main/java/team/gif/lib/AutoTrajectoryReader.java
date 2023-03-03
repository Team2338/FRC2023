package team.gif.lib;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

public class AutoTrajectoryReader {
    public static Trajectory generateTrajectoryFromFile(String filePath, TrajectoryConfig config) {
        try {
            Path traj_path = Filesystem.getDeployDirectory().toPath().resolve(filePath);
            TrajectoryGenerator.ControlVectorList control_vectors = WaypointReader.getControlVectors(traj_path);

            return TrajectoryGenerator.generateTrajectory(control_vectors, config);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + filePath, ex.getStackTrace());
            return null;
        }

    }
}
