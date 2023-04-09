package team.gif.lib;

import edu.wpi.first.math.controller.PIDController;

public class AutoPIDConfig {
    public final PIDController xController;
    public final PIDController yController;
    public final PIDController thetaController;

    public AutoPIDConfig(PIDController xController, PIDController yController, PIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
    }
}
