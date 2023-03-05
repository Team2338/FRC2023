package team.gif.lib.path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.util.ArrayList;

/**
 * @author rohancherukuri
 * @since 3/5/23
 * @version 1.0
 */
public class TrajectoryHandler {
    private static TrajectoryHandler instance;

    private PathPlannerTrajectory currentTrajectory;
    public boolean hasInitialPose = false;

    private TrajectoryHandler() {}

    public static TrajectoryHandler getInstance() {
        if (instance == null) {
            instance = new TrajectoryHandler();
        }

        return instance;
    }

    /**
     *
     * @param pathName
     * @param isReversed
     * @param maxVelocity
     * @param maxAcceleration
     * @return
     */
    public PathPlannerTrajectory newTrajectory (
        String pathName,
        boolean isReversed,
        double maxVelocity,
        double maxAcceleration
    ) {
        currentTrajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration, isReversed);
        return currentTrajectory;
    }

    /**
     *
     * @return
     */
    public PathPlannerTrajectory getCurrentTrajectory() {
        return currentTrajectory;
    }

    /**
     *
     * @return
     */
    public ArrayList<PathPlannerTrajectory.PathPlannerState> getStatesFromTrajectory() {
        int i = 1;
        ArrayList<PathPlannerTrajectory.PathPlannerState> states = new ArrayList<>();

        do {
            states.add(currentTrajectory.getState(i));
            i++;
        } while (!currentTrajectory.getState(i).equals(currentTrajectory.getEndState()));

        return states;
    }
}
