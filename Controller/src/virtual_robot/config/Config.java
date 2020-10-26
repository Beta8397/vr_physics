package virtual_robot.config;

import javafx.scene.image.Image;

/**
 * Class for configuring field (width and image), and gamepad (virtual vs. real)
 */
public class Config {

    /**
     *  Width of the subscene, in pixels
     */
    public static final double SUBSCENE_WIDTH = 800;

    /**
     * Whether to use "Virtual Gamepad" (true -> Virtual gamepad, false -> Real gamepad)
     */
    public static final boolean USE_VIRTUAL_GAMEPAD = false;

    /**
     * Enum for the Game
     */
    public enum Game {SKYSTONE, ULTIMATE_GOAL}

    /*
     * Currently active game
     */
    public static final Game GAME = Game.ULTIMATE_GOAL;

    /**
     * The image object for the field.
     */
    public static final Image BACKGROUND = GAME == Game.ULTIMATE_GOAL?
            new Image("/virtual_robot/assets/ultimate_goal_648.bmp") :
            new Image("/virtual_robot/assets/skystone_field648.bmp");


    /*
     * Behavior of virtual gamepad analog controls when they are released.
     *
     * false -> controls "snap back" to zero
     * true -> controls hold their position
     *
     * Either of these behaviors can be overridden by pressing SHIFT or ALT when control is released.
     */
    public static final boolean HOLD_CONTROLS_BY_DEFAULT = true;
}
