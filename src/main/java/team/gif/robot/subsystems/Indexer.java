package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

/**
 * Controls all aspects of the arm
 */
/*
 * 0 position is approx compass heading of 350
 *    (offsets are used in constants to take into account straight up being non-zero)
 * 0 degrees is considered straight up
 */
public class Indexer extends SubsystemBase {

    public static WPI_TalonSRX indexerMotor;

    public Indexer() {
        indexerMotor = new WPI_TalonSRX(RobotMap.INDEXER_MOTOR);
    }

    /**
     * Move the arm with an input current
     * @param percent percent of max current to move the arm
     */
    public void advance(double percent) {
        indexerMotor.set(percent);
    }
}
