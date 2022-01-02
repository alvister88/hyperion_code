package org.firstinspires.ftc.teamcode;

public class Params_Common {
    //////////////////// GENERAL SHOOTING VALUES /////////////////
    static final double INDEXER_EXTENDED_VALUE = 0.39;
    static final double INDEXER_RETRACTED_VALUE = 0.20;
    static final double INDEXER_FLUSH_VALUE = 0.50;

    static final int BASE_SHOOTER_RPM = 2200;

    //////////////////// WOBBLE CLAW VALUES //////////////////////
    static final double WB_CLAW_INIT = 0.29;
    static final double WB_CLAW_OPEN = 0.85;
    static final double WB_CLAW_CLOSE = 0.30;
    // position for letting the wobble drop down in init position
    static final double WB_CLAW_LOOSEN = 0.40;
    
    //////////////////// WOBBLE ARM VALUES ///////////////////////
    // init - initial position.
    // init_intermediate - when moving wobble to initial position, we bring it partway (to this position), before slowly moving it to the final position.
    // grab - position for grabbing the wobble.
    // grab_hover - when lifting a wobble near a wall, the wobble can collide with the wall.
    //              when we grab the wobble, we first lift it a tiny bit and back up, so that we are clear of the wall
    // drop - position for dropping the wobble over the wall.
    static final int WB_ARM_INIT = 0 - 50;
    static final int WB_ARM_INIT_INTERMEDIATE = -280 - 50;
    static final int WB_ARM_GRAB = -1675 - 50;
    static final int WB_ARM_GRAB_HOVER = -1575 - 50;
    static final int WB_ARM_DROP = -840 - 50;

    //////////////////// OTHER VALUES ////////////////////////////
    static final double BUMPER_SERVO_DOWN = 0.20;
    static final double BUMPER_SERVO_UP = 0.57;

    //////////////////// BLOCK ARM VALUES ////////////////////////
    static final double BLOCK_ARM_INIT_R = 0.12;
    static final double BLOCK_ARM_INIT_L = 0.81;

    static final double BLOCK_ARM_UP_WB_R = 0.3;
    static final double BLOCK_ARM_UP_WB_L = 0.81;

    static final double BLOCK_ARM_UP_R = 0.4;
    static final double BLOCK_ARM_UP_L = 0.7;

    static final double BLOCK_ARM_DOWN_R = 0.615;
    static final double BLOCK_ARM_DOWN_L = 0.45;
}
