// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.autoMode;
import team.gif.robot.commands.autos.NoAuto;
import team.gif.robot.commands.autos.PlaceCollectPlaceBarrier;
import team.gif.robot.commands.autos.PlaceCollectPlaceCable;
import team.gif.robot.commands.autos.PlaceCollectPlaceEngageCenter;
import team.gif.robot.commands.autos.PlaceCubeHighEngage;
import team.gif.robot.commands.autos.PlaceCubeHighMobility;
import team.gif.robot.commands.autos.PlaceCubeHighMobilityEngage;
import team.gif.robot.commands.autos.PlaceCubeHighMobilityEngagePP;
import team.gif.robot.commands.autos.PlaceCubeHighNoHomeEngage;
import team.gif.robot.commands.autos.PlaceMobilityEngageBarrier;
import team.gif.robot.commands.autos.PlaceMobilityEngageCable;
import team.gif.robot.commands.autos.ThreeGamePieceLeft;
import team.gif.robot.commands.autos.ThreeGamePieceRight;

import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final HashMap<autoMode, Command> autoCommands = new HashMap<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        buildAutoCommands();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    private void buildAutoCommands() {
        autoCommands.put(autoMode.NONE, new NoAuto());
        autoCommands.put(autoMode.PLACE_CUBE_HIGH_ENGAGE, new PlaceCubeHighEngage());
        autoCommands.put(autoMode.PLACE_CUBE_HIGH_NO_HOME_ENGAGE, new PlaceCubeHighNoHomeEngage());
        autoCommands.put(autoMode.PLACE_CUBE_HIGH_MOBILITY, new PlaceCubeHighMobility());
        autoCommands.put(autoMode.PLACE_CUBE_HIGH_MOBILITY_ENGAGE, new PlaceCubeHighMobilityEngagePP());
        autoCommands.put(autoMode.PLACE_COLLECT_PLACE_CABLE, new PlaceCollectPlaceCable());
        autoCommands.put(autoMode.PLACE_COLLECT_PLACE_BARRIER, new PlaceCollectPlaceBarrier());
        autoCommands.put(autoMode.PLACE_MOBILITY_ENGAGE_CABLE, new PlaceMobilityEngageCable());
        autoCommands.put(autoMode.PLACE_MOBILITY_ENGAGE_BARRIER, new PlaceMobilityEngageBarrier());
        autoCommands.put(autoMode.THREE_GP_RIGHT, new ThreeGamePieceRight());
        autoCommands.put(autoMode.THREE_GP_LEFT, new ThreeGamePieceLeft());
        autoCommands.put(autoMode.PLACE_COLLECT_PLACE_ENGAGE_CENTER, new PlaceCollectPlaceEngageCenter());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(autoMode chosenAuto) {
        Command autonomousCommand = autoCommands.get(chosenAuto);

        if (chosenAuto == null) {
            System.out.println("Autonomous selection is null. Robot will do nothing in auto :(");
        }

        return autonomousCommand;
    }
}
