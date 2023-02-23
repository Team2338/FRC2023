package team.gif.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GoLocationFromHome extends SequentialCommandGroup {
    public GoLocationFromHome() {
        addCommands(
            new GoLocationFromHomeArmStage(),
            new GoLocation()
        );
    }
}
