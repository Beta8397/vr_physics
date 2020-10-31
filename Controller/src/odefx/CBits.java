package odefx;

import org.ode4j.ode.DGeom;

public class CBits {

    /**
     * Category Bits
     */

    //Skystone Field Parts
    public static final long FLOOR = 0x1;
    public static final long WALLS = 0x2;
    public static final long BRIDGE = 0x4;

    //SkyStone Game Elements
    public static final long STONES = 0x10;
    public static final long FOUNDATIONS = 0x20;

    //Ultimate Goal Game Elements
    public static final long RINGS = 0x40;
    public static final long WOBBLES = 0x20;

    //General Bot
    public static final long BOT = 0x100;
    public static final long BOT_BOTTOM = 0x200;

    //BetaBot
    public static final long BOT_HANDS = 0x400;
    public static final long BOT_LEFT_HAND = 0x800;
    public static final long BOT_RIGHT_HAND = 0x1000;
    public static final long BOT_INTAKE = 0x2000;
    public static final long BOT_LEFT_INTAKE = 0x4000;
    public static final long BOT_RIGHT_INTAKE = 0x8000;
    public static final long BOT_INTAKE_ROOF = 0x10000;

    //UltimateBot
    public static final long BOT_RING_INTAKE = 0x400;
    public static final long SHOOTER = 0x2000;
    public static final long ARM = 0x800;
    public static final long HAND = 0x1000;

    //Two Wheel Bot
    public static final long BOT_FINGERS = 0x20000;
    public static final long BOT_LEFT_FINGER = 0x40000;
    public static final long BOT_RIGHT_FINGER = 0x80000;


    /**
     * Collide Bits
     */


}
