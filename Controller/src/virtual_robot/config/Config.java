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
     * The image object for the field.
     */
    public static final Image BACKGROUND = new Image("/virtual_robot/assets/ultimate_goal_648.bmp");

    /**
     * Enum for the Game
     */
    public enum Game {SKYSTONE, ULTIMATE_GOAL}

    /*
     * Currently active game
     */
    public static final Game GAME = Game.ULTIMATE_GOAL;
}
