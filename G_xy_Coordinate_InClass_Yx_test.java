/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.PI;

/**
 * .v01
 * Copy from Intake_Drive_Test_v00.
 * Initial changes
 ***************************************************************
 *   -------------------------------------------------------------
 *  . v04_09
 *  . remove reverse intake control bot drive back option.
 *  . auto turn on shooter motors when start to continue shooting. Still keep .b to switch shooter and intake function.
 *
 *  -------------------------------------------------------------
 *  . v05_00
 *  . import mw strafe drive code from mw_v04
 *
 *  -------------------------------------------------------------
 *  . v05_01
 *  . add block arms control.
 *  . relocate Left_Trigger+.a from configure drive power to set block arms to wobble pick position.
 *
 *  -------------------------------------------------------------
 *  v05_02
 *  . move block arms to from park for wobble pick up position when game start so they will not block shooting.
 *  . remove command wobbleCtrl.interrupt() from the end of main thread.
 *  . change condition for skip left stick control when pick up and drop wobble moving back.
 *  . after add delay in thread and main loop, DC problems improved.
 **  -------------------------------------------------------------
 *  *  v05_03
 *  *  . keep intake motor running when do high goal shooting
 *   -------------------------------------------------------------
 *  *  v05_04
 *  *  . shooter level pad changed. Retune paramteres for position A/B, single power shoot and continuous power shoot
 *
 *   -------------------------------------------------------------
 *  *  *  v06_00
 *  *  *  . use 40:1 wobble arm motor
 *   -------------------------------------------------------------
 *   v07_00
 *  . change turnCenter() ABS enable algorithm copy from VideoMake_claw_levle_black_shoot_v00.java
 *   -------------------------------------------------------------
 *   v08_00
 *   . move angle fine turn (gampad1.left/right) to wobble thread.
 *    -------------------------------------------------------------
 *    v09_00
 *    . remove wobbleCtrl thread. Target to improve DC issues.
 *    . remove static volatile type variable.
 * -------------------------------------------------------------
 *     v09_01
 *    . Relocate key: left_trigger+.y and left_trigger+.a
 *     -------------------------------------------------------------
 *      v09_02
 *    . wobble control state machine add position movements between init hold to wobble pick up.
 *    . change main loop delay timing from 30 to 50mS.
 *    . fine turn 3 rings power shooting 04/08
 *    -------------------------------------------------------------
 *      v09_02_mw
 *
 *    -------------------------------------------------------------
 *    v09_03
 *    . change wobble pick up servo close delay time from 400ms to 500ms.
 *    . wobble arm motor change back to 60:1
 *    . change parameter for 60:1 motor
 *
 *    -------------------------------------------------------------
 *    v09_04
 *   . add short horizontal movement for two parallel ring intake left trigger+ right stick_x
 *          Parallel Move and Turn (PMT move) to take parallel location rings.
 *   -------------------------------------------------------------
 *    v09_05
 *    . add bot stop for drop wobble either in init hold or end game hold position.
 *    . change main control loop cycle time from 50mS to 40mS.
 *    . add continue power shoot adjust offset GbPowerLevelAdj.
 *    -------------------------------------------------------------
 *     v09_05
 *    . change key assignment
 *    . change manual angle adjust step from 0.5 to 1 degrdd.
 *    . change manual levle adjust step from 0.025 to 0.01.
 *    . change end game position wobble delay from 500 to 200mS.
 *    . change wobble pick up claw close delay from 500 t0 400ms
 *    . enable Parallel Move and Turn (PMT move) function.
 *
 *   -------------------------------------------------------------
 *  v09_06
 *  . retune wobble delay parameters.
 *
 *   -------------------------------------------------------------
 *   v10_00
 *   .shooter track last pad changed. Bot more straight to tower goal.
 *   .re-tune auto high goal shooting
 *   .re-tune auto and manual power shooting.
 *  -------------------------------------------------------------
 *    v10_01
 *    . add PID modification.
 *  -------------------------------------------------------------
 *   v10_02
 *   .set shooter level to real position A level
 *   .make position A continue shooting interruptable by release the key between preset and shooting.
 *  -------------------------------------------------------------
 *   v10_03
 *   . fix wobble drop move back delay control bug. Remove unnecessary 500mS sleep.
 *   . reduce position A shooting shooter motor delay time from 0.35 to 0.2S.
 *   . add one more status wbInitPickUpPark for wobble slow target movement to init position.

 *  -------------------------------------------------------------
 *   v10_04
 *   . re-tune power shootings: level and tune angle.
 *
 * =========== Configuration file testConfigHW =============
 *
 * motorOdmR and motorOdmF for verify encoder port purpose.
 * */

@TeleOp(name="G_xy_Coordinate_InClass_Yx_test", group="TeleOp ...")
//@Disabled
public class G_xy_Coordinate_InClass_Yx_test extends LinearOpMode {
    /* Declare OpMode members. */
    //motors
    DcMotorEx motorRightFront;  //odometry right side odmR
    DcMotorEx motorRightBack;   //no encoder
    DcMotorEx motorLeftFront;  //odometry left side odmL
    DcMotorEx motorLeftBack;    //no encoder
    DcMotorEx motorShooterR;
    DcMotorEx motorShooterL;
    DcMotorEx motorWBClaw;
    DcMotorEx motorIntake;      //no encoder

    //servos
    Servo wobbleServo;
    Servo shootLevelServo;
    Servo shootServo;
    Servo blockArmR;
    Servo blockArmL;

    //Gyro
    BNO055IMU imu;
    // IMU variables
    double headingDoubleIMU = 0;
    Orientation angleIMU;

    static final double GbPowerLevelAdj = 0;
    //wobble claw servo
    static final double WOBBLECLAW_OPEN = 0.85;
    static final double WOBBLECLAW_CLOSE = 0.30;//0.33;
    static final double WOBBLECLAW_PARK = 0.29;//0.30;


    //wobble arm for 60:1 motor
    static final int WOBBLEARM_INIT     = 0;    // wobble teleOp park
//    static final int WOBBLEARM_HORIZON = -80;
    static final int WOBBLEARM_ENDGAME = -840;
    static final int WOBBLEARM_AUTOLIFT = -1550;
    static final int WOBBLEARM_PICKUP = -1675;

    /**
    //wobble arm for 40:1 motor
    static final int WOBBLEARM_INIT     = 0;    // wobble teleOp park
    static final int WOBBLEARM_HORIZON = -80;//-130;
    static final int WOBBLEARM_ENDGAME = -580;//-560;//-575;//-675;
    static final int WOBBLEARM_AUTOLIFT = -1025;//-1050;//-1100;//Low lift for autonomous
    static final int WOBBLEARM_PICKUP = -1120;//-1150;//-1200;
    */

    //Shooting speed
    static final int ShooterSpeed = 2200;//2100;

    //global IMU angle offset in degree
//    static volatile double GlobalAngleOffSet = 0;
//    static volatile double GlobalLevelOffset = 0;
//    static volatile int GlobalWobbleOffset = 0;
    double GlobalAngleOffSet = 0;
    double GlobalLevelOffset = 0;
    int GlobalWobbleOffset = 0;


    static final double shootLevel_A = 0.5;
    static final double shootLevel_B = 0.55;
    static final double shootLevel_C = 0.6;
    static final double shootLevel_D = 0.65;    //for power shoot

    static final double shootAngle_A = -12.5;//-12;
    static final double shootAngle_B = -2.5;//-1;
    static final double shootAngle_C = 15;
    static final double shootAngle_D = 0;   //for power shoot

    //shooting servo
    static final double SHOOTPARK = 0.19;//0.20;
    static final double SHOOTING = 0.40;//0.39;

    //shooting levle servo
    static final double SHOOTLEVELPARK = 0.75;

    //Power shooting
    static final double shootPwAngle_1 = 3;
    static final double shootPwAngle_2 = 7.5;//8;//5;
    static final double shootPwAngle_3 = 13.5;//14;//10;

    static final double ShootPwLevel_1 = 0.55;//0.56;//0.55;//0.54;//0.475;
    static final double ShootPwLevel_2 = 0.55;//0.56;//0.55;//0.54;
    static final double ShootPwLevel_3 = 0.55;//0.56;//0.55;//0.54;
    //Block arm positions
    static final double BlockArmParkR = 0.12;//0.15;//0.235;
    static final double BlockArmParkL = 0.81;

    static final double BlockArmWbPickR = 0.3;
    static final double BlockArmWbPickL = 0.81;

    static final double BlockArmUpR = 0.4;
    static final double BlockArmUpL = 0.7;

    static final double BlockArmDwnR = 0.615;
    static final double BlockArmDwnL = 0.45;

    public static final double NEW_P = 1.26;
    public static final double NEW_I = 0.126;
    public static final double NEW_D = 0.5;
    public static final double NEW_F = 12.6;

    static final double COUNTS_PER_ODM_REV = 1200; // Odometry counts 300x4
    //static final double COUNTS_PER_MOTOR_REV = 538;// neverRest 20:1//1120; // motor
    //static final double COUNTS_PER_MOTOR_REV = 538;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double ODM_DIAMETER_INCHES = 2.25;//2.5;//2.3;//2.25;     // For figuring circumference
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final int ODM_COUNTS_PER_INCH = (int) ((COUNTS_PER_ODM_REV * DRIVE_GEAR_REDUCTION) /
            (ODM_DIAMETER_INCHES * 3.1415));  // = 169.77
    static final double WHEEL_ODM_RATIO = WHEEL_DIAMETER_INCHES / ODM_DIAMETER_INCHES;
    static final double ODM_DISTANCE_RL = 13.5;      //inch

    //G_xy thread parameters
    public static final double OdmRL_Distance_inch = 6.75;//13.5;   //inch
    public static final double OdmB_Distance_inch = 6.25;
    public static final int OdmRL_Count_Per_inch = (int) ((COUNTS_PER_ODM_REV * DRIVE_GEAR_REDUCTION) /
            (ODM_DIAMETER_INCHES * 3.1415));
    public static final int OdmB_Count_Per_inch = (int) ((COUNTS_PER_ODM_REV * DRIVE_GEAR_REDUCTION) /
            (ODM_DIAMETER_INCHES * 3.1415));;
    public static final int UpdateTime = 10;   //mS

    double batteryVoltage;
    double batteryVoltageClass;
    // Computes the current battery voltage
    double getBatteryVoltage() {
        double voltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = sensor.getVoltage();
        }
        return voltage;
    }

    /**For test Hub 3 motors encoder ports which will be used for Odometry.
     *  Configuration file testConfigHW
     *
     */
    //DcMotorEx motorOdmR; // Right odometry. Hub 3 motor port 3
    //DcMotorEx motorOdmF; // Front odometry. Hub 3 motor port 2.

    int currentPositionIntake;
    boolean motorIntakeOnRdy = true;
    boolean motorIntakeOffRdy = false;


//    static volatile boolean LeftTriggerOn = false;
//    static volatile boolean leftBumperOn = false;
//    static volatile boolean RightTriggerOn = false;
//    static volatile boolean WobbleDriveOn = false;
    boolean LeftTriggerOn = false;
    boolean leftBumperOn = false;
    boolean RightTriggerOn = false;
    boolean WobbleDriveOn = false;

//    static volatile boolean blockArmUp;// = true;
//    static volatile boolean wbAdjUpEn = true;
//    static volatile boolean wbAdjDwnEn = true;

    boolean blockArmUp;// = true;
    boolean wbAdjUpEn = true;
    boolean wbAdjDwnEn = true;

    boolean levelUpAdjRdy = true;
    boolean levelDwnAdjRdy = true;

    double left_y;
    double left_x;
    double right_y;
    double right_x;

    /**
     * Status machine
     */
    boolean wbTelePark = true;
    boolean wbPickUp   = false;
    boolean wbHold     = false;
    boolean wbDrop     = false;
    boolean wbInitHold = false;
    boolean wbInitPickUpPark = false;

    double wbActionStartTime = 0;
    boolean wbInProcess = false;
    boolean grabbingZoneB = false;

    int wobbleCurrentPosition;
    double shootLevelCurrentPosition;


    @Override
    public void runOpMode(){

        motorLeftFront = hardwareMap.get(DcMotorEx.class, "MLF");//odometry R
        motorLeftBack = hardwareMap.get(DcMotorEx.class,"MLB");
        motorRightFront = hardwareMap.get(DcMotorEx.class,"MRF");//odometry L
        motorRightBack = hardwareMap.get(DcMotorEx.class,"MRB");


        motorShooterR = hardwareMap.get(DcMotorEx.class, "ShootR");
        motorShooterL = hardwareMap.get(DcMotorEx.class, "ShootL");

        motorIntake = hardwareMap.get(DcMotorEx.class, "Intake");
        motorWBClaw = hardwareMap.get(DcMotorEx.class, "WobbleClaw");

        shootLevelServo = hardwareMap.servo.get("ShootLevel");

        shootServo = hardwareMap.servo.get("shootServo");

        wobbleServo = hardwareMap.servo.get("WBGriper");

        blockArmL = hardwareMap.servo.get("blockArmL");
        blockArmR = hardwareMap.servo.get("blockArmR");


        //set motors direction
        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);  //odomety right side. Move forward odometry get negative counts
        motorRightBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorEx.Direction.FORWARD);   //odometry left side. Move forward odometry get negative counts
        motorLeftBack.setDirection(DcMotorEx.Direction.FORWARD);
        motorShooterR.setDirection(DcMotorEx.Direction.REVERSE);
        motorShooterL.setDirection(DcMotorEx.Direction.REVERSE);
        motorWBClaw.setDirection(DcMotorEx.Direction.FORWARD);
        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);

        //reset encoder
        motorRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  //odomety right side
        motorRightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);   //odometry left side
        //motorLeftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorShooterR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorShooterL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorWBClaw.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //drive train don't use encoder
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //set to use encoder
        motorShooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorShooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorWBClaw.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //set drive train motor stop behavior to float.
//        motorRightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorRightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorLeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorLeftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // IMU is attached to an I2C port on
        // a Core Device Interface Module and named "imu".
        // Set up the parameters with which will be used for IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData(">>>", "Calibrating IMU..");
        telemetry.update();

        //angle in degree
        angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        // double value headingDouble will use for Bot front direction
        headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                (angleIMU.angleUnit, angleIMU.firstAngle));


        boolean motorIntakeOn_Stick = true;
        boolean motorIntakeOn_Stick_Start = false;
        double motorIntakeOn_Stick_Start_Time = 0;

        double mrfPower;
        double mrbPower;
        double mlfPower;
        double mlbPower;

        double motorShootPrevTime = 0;

        //For drive train power scale factor. Practices purpose
        double trainPwFactor = 1;


        //set shooting servo to park position
        shootServo.setPosition(SHOOTPARK);

        //set wobble claw to park position
        wobbleServo.setPosition(WOBBLECLAW_PARK);
        //initial shooting level set to position A
        shootLevelServo.setPosition(shootLevel_A + GlobalLevelOffset - 0.1);
        shootLevelCurrentPosition = shootLevel_A + GlobalLevelOffset - 0.1;

//        shootLevelServo.setPosition(SHOOTLEVELPARK);    //0.75  //initial position

//        boolean levelUpAdjRdy = true;
//        boolean levelDwnAdjRdy = true;
        double stickAmp;

        boolean motorShooterOn = false;
        boolean motorShooterOnRdy = true;

        boolean motorIntakeOn = false;

        //For single shooting.
        boolean shootingRdy = true;

        //Power shooting
        boolean shootingPwRdy =true;
        int shootingPwCnt = 1;
        boolean shootingPwCntEn = true;

        boolean powerShootSingleEn = false;

        boolean circleDriveEn = false;
        int     circleDriveCnt = 0;

        boolean angleRightAdjRdy = true;
        boolean angleLeftAdjRdy = true;

        //PMT move
        boolean PMTmoveR = false;
        boolean PMTmoveL = false;

        //Block arm control variable. Block arm initial to up position
        blockArmL.setPosition(BlockArmParkL);
        blockArmR.setPosition(BlockArmParkR);
        blockArmUp = true;
        boolean blockArmMoveUpRdy = false;
        /**------------------------------------------------------*/

        //mw
        double turnAlloc = 0.4; // how much of max speed to allocate to turning
        double turnCoeff = 1.2; // coeff for turning, since i don't know how fast the robot will turn


        double shiftScale = 1.0;

        //        Create and start GlobalXYCoordinate thread to constantly update the global coordinate positions
        G_xy_Coordinate g_xy_Coordinate = new G_xy_Coordinate();
//        motorRightFront,
//                motorLeftFront,
//                motorRightBack,
//                OdmRL_Distance_inch,
//                OdmB_Distance_inch,
//                OdmRL_Count_Per_inch,
//                OdmB_Count_Per_inch,
//                UpdateTime);

//        Thread globalCoordinateUpdate = new Thread(g_xy_Coordinate);

//        WobbleCtrl wobbleCtrl = new WobbleCtrl();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        motorRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorRightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


//    public G_xy_Coordinate(DcMotorEx motorOdmR,
//                DcMotorEx motorOdmL,
//                DcMotorEx motorOdmB,
//                double imuHeadingMain,
//        double odmRL_Distance_inch,    //inch
//        double odmB_Distance_inch,
//        int odmRL_Count_Per_inch,
//        int odmB_Count_Per_inch,
//        int updateTime)

        //Start threads.
        g_xy_Coordinate.start();
//        globalCoordinateUpdate.start();

        batteryVoltage = getBatteryVoltage();

        //move block arms to pick up wobble position and they will not block shooting.
        blockArmL.setPosition(BlockArmWbPickL);
        blockArmR.setPosition(BlockArmWbPickR);
        blockArmUp = true;

        PIDFCoefficients pidOrig = motorShooterR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooterR.setVelocityPIDFCoefficients(NEW_P,NEW_I,NEW_D,NEW_F);
        motorShooterR.setPositionPIDFCoefficients(5.0);
//             re-read coefficients and verify change.
//            PIDFCoefficients pidModified = motorShooterR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        double prev_mainLoopStartTime = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //angle in degree
            angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES);
            // double value headingDouble will use for Bot front direction
            headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                    (angleIMU.angleUnit, angleIMU.firstAngle));

            /****************************************************************************************/
            // Display it for the driver.
            telemetry.addData("Global OdmR: ", "%7d", g_xy_Coordinate.getThreadOdmRTick());
            telemetry.addData("Global OdmL: ", "%7d", g_xy_Coordinate.getThreadOdmLTick());
            telemetry.addData("Global OdmB: ", "%7d", g_xy_Coordinate.getThreadOdmBTick());
            telemetry.addData(" ", " ");

            telemetry.addData("Global Heading: ", "%7.3f", g_xy_Coordinate.getGlobalHeadingDegree());
            telemetry.addData("Global Xg: ", "%7.3f", g_xy_Coordinate.getGlobalPos_X());
            telemetry.addData("Global Yg: ", "%7.3f", g_xy_Coordinate.getGlobalPos_Y());

            telemetry.addData(" ", " ");
            telemetry.addData("Cycle Time Avg (mS): ", "%7.4f", g_xy_Coordinate.getElapsedTimeAvg()*1000);
            telemetry.addData("Cycle Time Max(mS): ", "%7.4f", g_xy_Coordinate.getElapsedTimeMax()*1000);
            telemetry.addData("Cycle Time Min(mS): ", "%7.4f", g_xy_Coordinate.getElapsedTimeMin()*1000);

            telemetry.addData(" ", " ");
            telemetry.addData("IUM Gyro: ", "%5.2f", headingDoubleIMU);
            telemetry.addData("Main Loop Cycle Time(mS): ", "%7.4f", (getRuntime() - prev_mainLoopStartTime)*1000 );
            telemetry.update();
            prev_mainLoopStartTime = getRuntime();

            /****************************************************************************************/

            LeftTriggerOn = Math.abs(gamepad1.left_trigger) > 0.5;
            RightTriggerOn = Math.abs(gamepad1.right_trigger) > 0.1;
            
            left_y = -gamepad1.left_stick_y;// / 3;    //reverse y direction for motor.
            left_x = gamepad1.left_stick_x;/// 3;    //make half power for testing.

            right_y = -gamepad1.right_stick_y;// / 3;    //reverse y direction for motor.

            if (!LeftTriggerOn) {
                //By pass for PMT move
                right_x = gamepad1.right_stick_x;/// 3;    //make half power for testing.
            }

            LeftTriggerOn = Math.abs(gamepad1.left_trigger) > 0.5;
            RightTriggerOn = Math.abs(gamepad1.right_trigger) > 0.1;

            if(RightTriggerOn) {
                shiftScale = 1.0-(gamepad1.right_trigger * 0.7);
            } else {
                shiftScale = 1.0;
            }

            // sprint - power will be distributed based on translation and rotation and then scaled accordingly.
            // this means that turning is limited by movement speed, and vice versa.
            mlfPower = left_y+left_x + turnCoeff * right_x;
            mlbPower = left_y-left_x + turnCoeff * right_x;
            mrfPower = left_y-left_x - turnCoeff * right_x;
            mrbPower = left_y+left_x - turnCoeff * right_x;

            mlfPower *=  shiftScale;
            mlbPower *=  shiftScale;
            mrfPower *=  shiftScale;
            mrbPower *=  shiftScale;

            if(Math.abs(mlfPower) > 1.0 || Math.abs(mlbPower) > 1.0 || Math.abs(mrfPower) > 1.0 || Math.abs(mrbPower) > 1.0) {
                double maxVal = Math.max(Math.max(Math.abs(mlfPower), Math.abs(mlbPower)), Math.max(Math.abs(mrfPower), Math.abs(mrbPower)));
                mlfPower/=maxVal;
                mrbPower/=maxVal;
                mrfPower/=maxVal;
                mrbPower/=maxVal;
            }

            if (  !motorIntakeOn_Stick     //skip left stick control and use intake reverse drive back control
                       && !WobbleDriveOn)       //skip left stick control and use wobble drive back control
            {
                //drive train motor power controlled by left stick
                motorRightFront.setPower(mrfPower);
                motorRightBack.setPower(mrbPower);
                motorLeftFront.setPower(mlfPower);
                motorLeftBack.setPower(mlbPower);
            }

            //Free single shooting. Need turn on shooter motor by press .b. No other control applied.
            if (gamepad1.right_bumper && motorShooterOn
                    && !LeftTriggerOn   //continue power shooting
                    && !leftBumperOn)   //single power shooting with position selectable
            {

                //Disable single power shooting.
                if (powerShootSingleEn){
                    powerShootSingleEn = false;
                }

                if (shootingRdy ) {
                    shootServo.setPosition(SHOOTING);   //0.45
                    sleep(250);//300);
                    //park position
                    shootServo.setPosition(SHOOTPARK);  //0.20
                    sleep(180);//100);//150);

                    shootingRdy = false;
                }
            }
            else{
                shootingRdy = true;
            }

            //Continue power shooting.
            if (LeftTriggerOn){//gamepad1.left_trigger > 0.5){
//                LeftTriggerOn = true;
                if (gamepad1.right_bumper){ //) && motorShooterOn){
                    //turn off intake
                    motorIntake.setPower(0);
                    if (shootingPwCnt < 3) {
                        //turn on shooter
                        motorShooterR.setVelocity(ShooterSpeed - 200);//300);
                        motorShooterL.setVelocity(ShooterSpeed - 200);//300);
                        motorShooterOn = true;
                    }

                    //wait for shooter motors stable
                    sleep(200); //for motor to stable

//                    if (shootingPwRdy ) {
                        if (shootingPwCnt == 1) {
                            //set shooting level. Fixed without adjust by GlobalLevelOffset
                            shootLevelServo.setPosition(ShootPwLevel_1  -0.13);//-0.125);//-0.13);//-0.135);
                            shootLevelCurrentPosition = ShootPwLevel_1  -0.13;//-0.125;//-0.13;//-0.135;
//                            shootLevelServo.setPosition(ShootPwLevel_1 - 0.0975);//0.095);
                            //turn bot to next power shoot angle
//                            turnCenter(0.55,//0.6,//0.65,
//                                    (shootPwAngle_1 + GlobalAngleOffSet +0.5),//+1),//+4),//+3),//+2.5),
//                                    5, 0.75,//0.5,//0.75,
//                                    5,//1,
//                                    1);
                            turnCenter(0.55,//0.6,//0.5,
                                    (shootPwAngle_3 + GlobalAngleOffSet +1.5),//+1),//+0.5),
                                    5, 0.75,//0.5,//0.75,
                                    5,
                                    1);
                            if (motorShooterOn && gamepad1.right_bumper) {
                                shootServo.setPosition(SHOOTING);   //0.45
                                //Low block arms after fist shooting
                                blockArmL.setPosition(BlockArmDwnL);
                                blockArmR.setPosition(BlockArmDwnR);
                                blockArmUp = false;//true;

                                sleep(250);

                                //shoot arm to park position don't need to wait.
                                shootServo.setPosition(SHOOTPARK);  //0.20
//                                sleep(150);
                                shootingPwCnt++;
                            }
                        }
                        else if (shootingPwCnt == 2) {
                            //set 2nd shooting level. Don't need set because it is same as 1st.
//                            shootLevelServo.setPosition(ShootPwLevel_2  -0.135);//-0.13);//+ GlobalLevelOffset - 0.12);//0.075);
//                            shootLevelCurrentPosition = ShootPwLevel_2  -0.135;//-0.13;//+ GlobalLevelOffset - 0.12;
                            //turn bot to next power shoot angle
                            turnCenter(0.55,//0.6,//0.5,
                                    (shootPwAngle_2  + GlobalAngleOffSet +1.35),//+1),//+1.5),//+4.5),//+3.5),//+3),
                                    5, 0.75,//0.5,//0.75,
                                    5,
                                    1);
                            if (motorShooterOn && gamepad1.right_bumper) {
                                shootServo.setPosition(SHOOTING);   //0.45
                                sleep(250);
                                //park position
                                shootServo.setPosition(SHOOTPARK);  //0.20
//                                sleep(150);
                                shootingPwCnt++;
                            }
                        }
                        else if (shootingPwCnt == 3) {
                            //set 3rd shooting level. Don't need set because it is same as 1st.
//                            shootLevelServo.setPosition(ShootPwLevel_3   -0.135);//-0.13);// + GlobalLevelOffset - 0.11);//0.07);
//                            shootLevelCurrentPosition = ShootPwLevel_3   -0.135;//-0.13;//+ GlobalLevelOffset - 0.11;
                            //turn bot to next power shoot angle
                            turnCenter(0.55,//0.6,//0.65,
                                    (shootPwAngle_1 + GlobalAngleOffSet +0.5),//+1),//+4),//+3),//+2.5),
                                    5, 0.75,//0.5,//0.75,
                                    5,//1,
                                    1);
//                            turnCenter(0.55,//0.6,//0.5,
//                                    (shootPwAngle_3 + GlobalAngleOffSet +0.5),//+1),//+4),//+3),//+2.5),
//                                    5, 0.75,//0.5,//0.75,
//                                    5,
//                                    1);
                            if (motorShooterOn && gamepad1.right_bumper) {
                                shootServo.setPosition(SHOOTING);   //0.45
                                sleep(250);//300);
                                //park position
                                //continue power shoot finish, turn off shooter motor and turn on intake
                                motorShooterR.setPower(0);
                                motorShooterL.setPower(0);
                                motorShooterOn = false;
                                shootServo.setPosition(SHOOTPARK);  //0.20
//                                sleep(150);
                                shootingPwCnt++;
                            }
                        }
//                        sleep(150);
//                        shootingPwRdy = false;
//                    }
                }
                else{
                    shootingPwRdy = true;
                    if (shootingPwCnt > 3){
                        //reset shooting counter
                        shootingPwCnt = 1;
                        //continue power shoot finish, turn off shooter motor and turn on intake
                        motorShooterR.setPower(0);
                        motorShooterL.setPower(0);
                        motorShooterOn = false;

                        //turn on intake
                        //bug: when LeftTriggerOn changed before right_bumper release. This route doesn't
                        //executed. It will execute when push left trigger and turn on the intake.
                        motorIntake.setPower(1);
                        motorIntakeOn = true;
                    }
                }
            }

            //Single power shooting. Shooting level will adjusted by GlobalAngleOffSet.
            //Select power shooting location. In case some power shooting missed in continuously shooting.
            //After adjust power shoot position, use single shooting control "right_bumper" to shoot.
            if (gamepad1.left_bumper){
                leftBumperOn = true;
                //one press increase 1 for counter
                if (gamepad1.right_bumper && shootingPwCntEn){
                    //enable single power shoot motor speed.
                    powerShootSingleEn = true;
                    //turn off intake
                    motorIntake.setPower(0);

                    //turn one power shoot motors. Subtract from normal high goal shooting speed.
                    motorShooterR.setVelocity(ShooterSpeed- 200);//300);
                    motorShooterL.setVelocity(ShooterSpeed- 200);//300);
                    motorShooterOn = true;


                    //all 3 power use same level
                    shootLevelServo.setPosition(ShootPwLevel_1 + GlobalLevelOffset -0.13);//-0.125);//-0.135);//-0.13);
                    shootLevelCurrentPosition = ShootPwLevel_1 + GlobalLevelOffset -0.13;//-0.125;//-0.135;//-0.13;

                    if(shootingPwCnt == 1){
                        //set shooting level. Fixed without adjust by GlobalLevelOffset
//                        shootLevelServo.setPosition(ShootPwLevel_1 + GlobalLevelOffset -0.13);//-0.125);//-0.135);//-0.13);
//                        shootLevelCurrentPosition = ShootPwLevel_1 + GlobalLevelOffset -0.13;//-0.125;//-0.135;//-0.13;
                                //turn bot to next power shoot angle
                        turnCenter(0.6,//0.55,
                                (shootPwAngle_1 + GlobalAngleOffSet  +0.5),//+3),//+2.5),//-2),//-2.5),
                                5, 0.75,//0.5,//0.75,
                                5,
                                1);
                        shootingPwCnt++;
                        shootingPwCntEn = false;

                        //Low block arms after fist shooting
//                        blockArmL.setPosition(BlockArmDwnL);
//                        blockArmR.setPosition(BlockArmDwnR);
//                        blockArmUp = false;//true;

                    }
                    else if (shootingPwCnt == 2){
                        //set 2nd shooting level.
//                        shootLevelServo.setPosition(ShootPwLevel_2 + GlobalLevelOffset -0.13);//-0.125);//-0.135);//-0.13);//- 0.125);//- 0.12);//0.075);
//                        shootLevelCurrentPosition = ShootPwLevel_2 + GlobalLevelOffset -0.13;//-0.125;//-0.135;//-0.13;//- 0.125;//- 0.12;
                        //turn bot to next power shoot angle
                        turnCenter(0.5,
                                (shootPwAngle_2  + GlobalAngleOffSet +1),//+3.5),//+3),//-1.5),//-0.5),
                                5, 0.75,//0.5,//0.75,
                                5,
                                1);
                        shootingPwCnt++;
                        shootingPwCntEn = false;
                    }
                    else if (shootingPwCnt == 3){
                        //set shooting level
//                        shootLevelServo.setPosition(ShootPwLevel_3  + GlobalLevelOffset  -0.13);//-0.125);//-0.135);//-0.13);//- 0.115);//- 0.11);//0.07);
//                        shootLevelCurrentPosition = ShootPwLevel_3  + GlobalLevelOffset  -0.13;//-0.125;//-0.135;//-0.13;//- 0.115;//- 0.11;
                        //turn bot to next power shoot angle
                        turnCenter(0.5,
                                (shootPwAngle_3 + GlobalAngleOffSet +0.5),//+3),//+2.5),//- 1.25),//0.75),
                                5, 0.75,//0.5,//0.75,
                                5,
                                1);
                        shootingPwCnt++;
                        shootingPwCntEn = false;
                    }
                }else {
                    shootingPwCntEn = true;
                    if (shootingPwCnt > 3) {
                        shootingPwCnt = 1;
                    }
                }
            }
            else{
                leftBumperOn = false;
            }


            //block arm initial to park position
//            boolean blockArmUp = true;
//            boolean blockArmMoveUpRdy = false;
            if (gamepad1.y && !LeftTriggerOn) {
                if (blockArmMoveUpRdy) {
                    blockArmL.setPosition(BlockArmUpL);
                    blockArmR.setPosition(BlockArmUpR);
                    //block arm position
                    blockArmUp = true;
                } else {
                    blockArmL.setPosition(BlockArmDwnL);//0.0);
                    blockArmR.setPosition(BlockArmDwnR);//1.0);
                    blockArmUp = false;
                }
            }
            else{
                if (blockArmUp) {
                    blockArmMoveUpRdy = false;
                }
                else {
                    blockArmMoveUpRdy = true;
                }
            }

            //Config drive train power pre-setting
            if (gamepad1.y && LeftTriggerOn) {
                //move block arms to pick up wobble position in case it accidentally move to other position.
                blockArmL.setPosition(BlockArmWbPickL);
                blockArmR.setPosition(BlockArmWbPickR);
                blockArmUp = true;
            }
//                if (gamepad1.x){
//                    trainPwFactor = 0.5;
//                }
//                if (gamepad1.a){
//                    trainPwFactor = 0.7;

//                }
//                if (gamepad1.b){
//                    trainPwFactor = 0.85;
//                }
//                if (gamepad1.y){
//                    trainPwFactor = 1.0;
//                    move block arms to pick up wobble position in case it accidentally move to other position.
//                    blockArmL.setPosition(BlockArmWbPickL);
//                    blockArmR.setPosition(BlockArmWbPickR);
//                    blockArmUp = true;
//                }
//            }

            //forward/backward intake motor using right stick y
            if (!LeftTriggerOn) {  //multi function: right_y also control wobble adjust
                if ((-gamepad1.right_stick_y) >= 0.5) {
                    motorIntake.setPower(1);
                    motorIntakeOn_Stick_Start_Time = 0;
                    //for intake controlled by left bumper and double click right bumper
                    motorIntakeOn = true;
                }
                else if ((-gamepad1.right_stick_y) <= -0.5) {
                    if (motorIntakeOn_Stick_Start_Time == 0) {
                        motorIntakeOn_Stick_Start_Time = getRuntime();
                        motorIntakeOn_Stick_Start = false;
                    }
                    //Hold more than 0.3S, reverse intake.
                    if ((getRuntime() - motorIntakeOn_Stick_Start_Time) > 0.25){//0.3) {
                        motorIntake.setPower(-1);
                        //drive bot back while drive on left stick
                        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                            motorRightFront.setPower(-0.4);//-0.5);
                            motorRightBack.setPower(-0.4);//-0.5);
                            motorLeftFront.setPower(-0.4);//-0.5);
                            motorLeftBack.setPower(-0.4);//-0.5);
                            motorIntakeOn_Stick = true; //only turn off motorIntake.setPower(-1) condition.
                        }
                        else{
                            //to use left stick x.y. control bot while intake working in reverse.
                            motorIntakeOn_Stick = false;
                        }
                    } else {
                        motorIntake.setPower(0);
//                        motorIntakeOn_Stick_Start_Time = 0;
                        //for intake controlled by left bumper and double click right bumper
                        motorIntakeOn = false;
                    }
//                    motorIntakeOn_Stick = true; //only turn off motorIntake.setPower(-1) condition.
                }
//                else if ((Math.abs(gamepad1.right_stick_y) < 0.5)
//                        && (motorIntakeOn_Stick)) {
                else if ((Math.abs(gamepad1.right_stick_y) < 0.5)){
                    motorIntakeOn_Stick_Start_Time = 0;
                    if (motorIntakeOn_Stick) {
                        //it will not turn off intake when turn on by .b button
                        motorIntake.setPower(0);

                        //stop bot
                        //If left stick already take over control, skip set 0 power
                        if (gamepad1.left_stick_x != 0 && gamepad1.left_stick_y != 0) {
                            motorRightFront.setPower(0);
                            motorRightBack.setPower(0);
                            motorLeftFront.setPower(0);
                            motorLeftBack.setPower(0);
                        }
                        //for intake controlled by button .b
                        motorIntakeOn = false;
//                    motorIntakeOn_Stick_Start_Time = 0;
                        motorIntakeOn_Stick = false;
                    }
                }
            }

            //turn on/off intake motor or shooter motors alternatively.
//            if (gamepad1.left_bumper) {
            if (gamepad1.b && !LeftTriggerOn && !leftBumperOn) {

                if (motorShooterOnRdy ) {
                    motorShooterR.setVelocity(ShooterSpeed);
                    motorShooterL.setVelocity(ShooterSpeed);
//                    motorShooterR.setPower(1);
//                    motorShooterL.setPower(1);
                    motorShooterOn = true;
                    powerShootSingleEn = false;

                    //turn off intake
                    motorIntake.setPower(0);
                    motorIntakeOn = false;
                }
                else {
                    motorShooterR.setPower(0);
                    motorShooterL.setPower(0);
                    motorShooterOn = false;
                    powerShootSingleEn = false;

                    //turn on intake
                    motorIntake.setPower(1);
                    motorIntakeOn = true;
                }
            }
            else{
                if (motorShooterOn){
                    motorShooterOnRdy = false;
                }
                else{
                    motorShooterOnRdy = true;
                }
            }

            //Position A shooting
            if (RightTriggerOn){//gamepad1.right_trigger > 0.5){
//                RightTriggerOn = true;
                //Shooting position A
                if(gamepad1.dpad_left){
                    //turn off intake
//                    motorIntake.setPower(0);
                    //turn on shooter motors
                    if (motorShootPrevTime == 0){
                        motorShootPrevTime = getRuntime();
                        //turn on shooter motors when start to continue shooting
                        motorShooterR.setVelocity(ShooterSpeed-100);    //reduce speed and increase levle
                        motorShooterL.setVelocity(ShooterSpeed-100);
                        motorShooterOn = true;
                    }
                    //disable single power shoot control
                    powerShootSingleEn = false;

                    //set shoot level
                    shootLevelServo.setPosition(shootLevel_A + GlobalLevelOffset - 0.1);//-0.105);
                    shootLevelCurrentPosition = shootLevel_A + GlobalLevelOffset -0.1;//-0.105;
                    //turn bot to shoot angle
                    turnCenter(0.6,//0.5,
                            ( shootAngle_A + GlobalAngleOffSet +9),
                            5,
                            1.0,
                            5,//2,
                            1);

                    //wait for shooter motors stable
                    while ( ! ((getRuntime() - motorShootPrevTime) > 0.35))//0.2))//0.35))//0.30))  //reduce waiting time
                    {
                    }

                    if(gamepad1.dpad_left && motorShooterOn) {
                        //If gamepad1.dpad_left still hold on and shooter motors are on, then shooting
                        //Low block arms after fist shooting
                        blockArmL.setPosition(BlockArmDwnL);
                        blockArmR.setPosition(BlockArmDwnR);
                        blockArmUp = false;//true;

                        //continue 3 shoots
                        shootServo.setPosition(SHOOTING);//0.40);//0.45);

                        sleep(200);//220);//180);//200);//230);
                        //park position
                        shootServo.setPosition(SHOOTPARK);//+0.02);//0.18);
                        sleep(200);//180);//160);//180);//200);

//                        if (gamepad1.dpad_left && motorShooterOn) {
                        //If gamepad1.dpad_left still hold on and shooter motors are on, then shooting
                        shootServo.setPosition(SHOOTING);//0.40);//0.45);
                        sleep(200);//220);//160);//200);//230);
                        //park position
                        shootServo.setPosition(SHOOTPARK);//+0.02);//0.18);
                        sleep(220);//180);//160);//180);//200);

//                       if (gamepad1.dpad_left && motorShooterOn) {
                        //If gamepad1.dpad_left still hold on and shooter motors are on, then shooting
                        shootServo.setPosition(SHOOTING);//0.40);//0.45);
                        sleep(200);//220);//160);//200);//230);
                        //park position
                        shootServo.setPosition(SHOOTPARK);//0.18);
//                            }
//                        }
                    }
                    //After shooting, turn off shooter and turn on intake(always ON).
                    motorShooterR.setPower(0);
                    motorShooterL.setPower(0);
                    motorShooterOn = false;

                    //reset shooter motor turn on start time when finish shooting
                    motorShootPrevTime = 0;
                    //turn on intake
                    motorIntake.setPower(1);
                    motorIntakeOn = true;
                }

                //Shooting position B
                else if (gamepad1.dpad_down){

                    //turn off intake
//                    motorIntake.setPower(0);

                    //turn on shooter motors
                    if (motorShootPrevTime == 0){
                        motorShootPrevTime = getRuntime();
                        //turn on shooter motors when start to continue shooting
                        motorShooterR.setVelocity(ShooterSpeed);
                        motorShooterL.setVelocity(ShooterSpeed);
                        motorShooterOn = true;
                    }
                    //disable single power shoot control
                    powerShootSingleEn = false;

                    //disable single power shoot control
                    powerShootSingleEn = false;
                    //set shoot level
                    shootLevelServo.setPosition( shootLevel_B + GlobalLevelOffset-0.09);//-0.08);
                    shootLevelCurrentPosition = shootLevel_B + GlobalLevelOffset-0.09;
                    //turn bot to shoot angle
                    turnCenter(0.6,//0.5,
                            (shootAngle_B + GlobalAngleOffSet ),//+ 0.025),
                            3,
                            0.75,
                            1,
                            1);
                    //wait for shoot motors stable
                    while ( ! ((getRuntime() - motorShootPrevTime) > 0.40))//0.3))
                    {

                    }

                    if (gamepad1.dpad_down && motorShooterOn) {
                        //continue 3 shoots
                        shootServo.setPosition(SHOOTING);//0.40);
                        //Low block arms after fist shooting
                        blockArmL.setPosition(BlockArmDwnL);
                        blockArmR.setPosition(BlockArmDwnR);
                        blockArmUp = false;//true;
                        sleep(230);
                        //park position
                        shootServo.setPosition(0.18);
                        //fine tune level for 2nd shooting (org)
                        shootLevelServo.setPosition( shootLevel_B + GlobalLevelOffset-0.07);//-0.08);
                        shootLevelCurrentPosition = shootLevel_B + GlobalLevelOffset-0.07;

                        sleep(180);//200);
                        shootServo.setPosition(SHOOTING);//0.40);
                        sleep(230);
                        //park position
                        shootServo.setPosition(0.18);
                        //fine tune levle for 3rd shooting
                        shootLevelServo.setPosition( shootLevel_B + GlobalLevelOffset-0.075);
                        shootLevelCurrentPosition = shootLevel_B + GlobalLevelOffset-0.075;
                        sleep(180);
                        shootServo.setPosition(SHOOTING);//0.40);
                        sleep(230);
                        //park position
                        shootServo.setPosition(0.18);


                        //After shooting, turn off shooter and turn on intake.
                        motorShooterR.setPower(0);
                        motorShooterL.setPower(0);
                        motorShooterOn = false;

                        //reset shooter motor turn on start time when finish shooting
                        motorShootPrevTime = 0;

                        //turn on intake
                        motorIntake.setPower(1);

                        motorIntakeOn = true;
                    }
                }
                else if (gamepad1.dpad_right){
                    //not used.
                    //disable single power shoot control
                    powerShootSingleEn = false;

                }
                else if (gamepad1.dpad_up){
                    // To reset angle offset: GlobalAngleOffSet for power shoot control
                    // 1> turn bot to parallel launch line (0 degree)
                    // 2> push RightTrigger button + gampad1.dpad_up to reset global angle offset
                    powerShootSingleEn = false;
                    //reset power shooting counter
                    shootingPwCnt = 1;
                    angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES);
                    // double value headingDouble will use for Bot front direction
                    headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                            (angleIMU.angleUnit, angleIMU.firstAngle));

                    //get new offset
                    GlobalAngleOffSet = headingDoubleIMU;
                }
            }


            else {
//                RightTriggerOn = false;
                //Manually adjust bot angle
//                boolean angleRightAdjRdy = true;
//                boolean angleLeftAdjRdy = true;
                if (gamepad1.dpad_right) {
                    //only change GlobalAngleOffset value without move bot
                    if (angleRightAdjRdy) {
                        GlobalAngleOffSet -= 2.5;//2;//1;//-= 0.5;
                        angleRightAdjRdy = false;
                    }
                } else {
                    angleRightAdjRdy = true;
                }

                if (gamepad1.dpad_left) {
                    //only change GlobalAngelOffset value without move bot
                    if (angleLeftAdjRdy) {
                        GlobalAngleOffSet += 2.5;//2;//1;//0.5;
                        angleLeftAdjRdy = false;
                    }
                } else {
                    angleLeftAdjRdy = true;
                }



    //            static final double shootLevel_A = 0.5;
    //            static final double shootLevel_B = 0.5;
    //            static final double shootLevel_C = 0.6;
    //            static final double shootLevel_D = 0.65;    //for power shoot
    //            GlobalAngleOffSet

                //Manually adjust shooting level up
                //All shooting position ABCD share same level offset
                if (gamepad1.dpad_up) {
                    //Use current position for reference.
                    //It will be optimized for the same shooting position.
                    // It may be acceptable for different positions.
                    if (levelUpAdjRdy) {
                        if ((shootLevel_A + GlobalLevelOffset) >= 0.1) {
                            GlobalLevelOffset -= 0.02;//0.01;//0.025;
                            shootLevelCurrentPosition -= 0.02;//0.01;//0.025;
                        }
                        levelUpAdjRdy = false;
                    }
                    //adjust shooting level for shooting position A
                    shootLevelServo.setPosition(shootLevelCurrentPosition);
//                    shootLevelCurrentPosition = shootLevelCurrentPosition;
                } else {
                    levelUpAdjRdy = true;
                }

                //adjust shooting level down
                if (gamepad1.dpad_down) {
                    if (levelDwnAdjRdy) {
                        if ((shootLevel_A + GlobalLevelOffset) <= 0.75) {
                            GlobalLevelOffset += 0.02;//0.01;//0.025;
                            shootLevelCurrentPosition += 0.02;//0.01;//0.025;
                        }
                        levelDwnAdjRdy = false;
                    }
                    //adjust shooting level for shooting position A
                    shootLevelServo.setPosition(shootLevelCurrentPosition);
//                    shootLevelCurrentPosition = shootLevelCurrentPosition;
                } else {
                    levelDwnAdjRdy = true;
                }
            }


            //set bot to current 0 degree reference
            if (gamepad1.b && !LeftTriggerOn && leftBumperOn) {
                //turn bot to current 0 degree reference angle
                turnCenter(0.5,//0.6,
                        (0 + GlobalAngleOffSet ),//+ 0.025),
                        3,
                        0.5,
                        2,
                        1);
            }
            //adjust direction for end game power shooting
            double GlobalAngleOffSetTemp = 0;
            if (gamepad1.b && LeftTriggerOn && !leftBumperOn) {

                //turn off intake
                motorIntake.setPower(0);
                //Manual adjust shooting angle need to update global shooing angle offset.
                //get IMU angle before adjust
                //angle in degree
                angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES);
                // double value headingDouble will use for Bot front direction
                headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                        (angleIMU.angleUnit, angleIMU.firstAngle));
                GlobalAngleOffSetTemp = headingDoubleIMU;

                motorRightFront.setPower(-0.2);
                motorRightBack.setPower(-0.2);
                motorLeftFront.setPower(0.2);
                motorLeftBack.setPower(0.2);
                sleep(50);
                motorRightFront.setPower(0);
                motorRightBack.setPower(0);
                motorLeftFront.setPower(0);
                motorLeftBack.setPower(0);
                sleep((150));
                angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES);
                // double value headingDouble will use for Bot front direction
                headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                        (angleIMU.angleUnit, angleIMU.firstAngle));
                //get new offset
                GlobalAngleOffSet += headingDoubleIMU - GlobalAngleOffSetTemp;

            }
            if (gamepad1.x && LeftTriggerOn) {
                //turn off intake
                motorIntake.setPower(0);
                //Manual adjust shooting angle need to update global shooing angle offset.
                //get IMU angle before adjust
                //angle in degree
                angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES);
                // double value headingDouble will use for Bot front direction
                headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                        (angleIMU.angleUnit, angleIMU.firstAngle));

                GlobalAngleOffSetTemp = headingDoubleIMU;

                motorRightFront.setPower(0.2);
                motorRightBack.setPower(0.2);
                motorLeftFront.setPower(-0.2);
                motorLeftBack.setPower(-0.2);
                sleep(50);
                motorRightFront.setPower(0);
                motorRightBack.setPower(0);
                motorLeftFront.setPower(0);
                motorLeftBack.setPower(0);
                sleep(150);

                angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES);
                // double value headingDouble will use for Bot front direction
                headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                        (angleIMU.angleUnit, angleIMU.firstAngle));

                //get new offset
                GlobalAngleOffSet += headingDoubleIMU - GlobalAngleOffSetTemp;
            }



/*****************************************************************************************************************/
//            while (opModeIsActive() &&
//                    !isInterrupted() && wbcIsRunning)
//            {
//                try {

                    //wobble arm manually adjust
                    if (LeftTriggerOn){
                        wobbleCurrentPosition = motorWBClaw.getCurrentPosition();
                        if (gamepad1.right_stick_y > 0.5 ){   //push down
                            if (wbAdjDwnEn){
                                GlobalWobbleOffset -= 80;//40;//25;
                                //wobble arm move to adjusted position
                                motorWBClaw.setTargetPosition(wobbleCurrentPosition - 80);//40 );//25);
                                motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                motorWBClaw.setPower(1.0);//0.8);
                                while(motorWBClaw.isBusy()){}
                                wbAdjDwnEn = false;
                            }
                        }
                        else {
                            wbAdjDwnEn = true;
                        }

                        if (gamepad1.right_stick_y < -0.5) {    //push up
                            if (wbAdjUpEn) {
                                GlobalWobbleOffset += 80;//40;//25;
                                //wobble arm move to adjusted position
                                motorWBClaw.setTargetPosition(wobbleCurrentPosition + 80);//40);//25);
                                motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                motorWBClaw.setPower(1.0);//0.8);
                                while (motorWBClaw.isBusy()) {}
                                wbAdjUpEn = false;
                            }
                        }
                        else {
                            wbAdjUpEn = true;
                        }
                    }
                    else{
                        wbAdjUpEn = true;
                        wbAdjDwnEn = true;
                    }
                    /*****************************************************************************************/

                    //State machine
                    if (wbTelePark){
                        if (gamepad1.a && !LeftTriggerOn){

                            //move block arms to pick up wobble position and they will not block shooting.
                            blockArmL.setPosition(BlockArmWbPickL);
                            blockArmR.setPosition(BlockArmWbPickR);
                            blockArmUp = true;

                            //wobble arm move to pick up position
                            motorWBClaw.setTargetPosition(WOBBLEARM_PICKUP + GlobalWobbleOffset); //800
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(1.0);//0.8);
                            //claw to open position
                            sleep(200);//400);//100);
                            wobbleServo.setPosition(WOBBLECLAW_OPEN);

                            //no waiting in maim loop
//                            while(motorWBClaw.isBusy()){
//                            }
                            //change status
                            wbPickUp = true;
                            wbTelePark = false;
                        }
                    }

                    else if (wbPickUp){
                        if (gamepad1.a && !LeftTriggerOn){
                            //claw close
                            wobbleServo.setPosition(WOBBLECLAW_CLOSE);

                            sleep(400);//500);//400);
                            //wobble arm lift. Wait for bot move back and then move to target.
                            //wobble arm move to hold position. Same position to drop the wobble
                            motorWBClaw.setTargetPosition(WOBBLEARM_PICKUP + 100 + GlobalWobbleOffset);//(WOBBLEARM_AUTOLIFT);//(900);
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(0.8);//0.4);

                            //drive bot back
                            while (gamepad1.left_stick_x == 0
                                    && gamepad1.left_stick_y == 0
                                    && gamepad1.right_stick_x == 0){
                                WobbleDriveOn = true;
                                //drive bot back
                                motorRightFront.setPower(-0.7);//-0.5);
                                motorRightBack.setPower(-0.7);//-0.5);
                                motorLeftFront.setPower(-0.7);//-0.5);
                                motorLeftBack.setPower(-0.7);//-0.5);
                            }

                            //wobble arm move to hold position. Same position to drop the wobble
                            motorWBClaw.setTargetPosition(WOBBLEARM_ENDGAME + GlobalWobbleOffset);//(WOBBLEARM_AUTOLIFT);//(900);
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(0.8);

                            WobbleDriveOn = false;
                            //no waiting in maim loop
//                            while(motorWBClaw.isBusy()){}
                            //change status
                            wbHold = true;
                            wbPickUp = false;
                        }
                        else if (gamepad1.a && LeftTriggerOn) {
                            //wobble arm move to INIT hold position.
                            //claw close
                            wobbleServo.setPosition(WOBBLECLAW_CLOSE);
                            sleep(300);//400);
                            //wobble arm move to hold position. Same position to drop the wobble
                            motorWBClaw.setTargetPosition(WOBBLEARM_INIT + GlobalWobbleOffset -350);
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(0.9);//0.8);

                            //no waiting in maim loop
//                            while(motorWBClaw.isBusy()){}
                            //change status and go to wbInitPickUpPark
                            wbInitPickUpPark = true;
//                            wbHold = true;
                            wbPickUp = false;

                                //drive bot back
//                            while (gamepad1.left_stick_x == 0
//                                    && gamepad1.left_stick_y == 0
//                                    && gamepad1.right_stick_x == 0){
//                                WobbleDriveOn = true;
//                                drive bot back
//                                motorRightFront.setPower(-0.5);
//                                motorRightBack.setPower(-0.5);
//                                motorLeftFront.setPower(-0.5);
//                                motorLeftBack.setPower(-0.5);
//                            }
                        }
                    }
                    else if (wbInitPickUpPark){
                        //change motor power when position > -320
                        if (motorWBClaw.getCurrentPosition() > -370) {
                            while (motorWBClaw.isBusy()) {
                            }
                            //small power go to target
                            motorWBClaw.setTargetPosition(WOBBLEARM_INIT + GlobalWobbleOffset);
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(0.1);

                            //change status and go to wbInitHold
                            wbInitHold = true;
//                            wbHold = true;
                            wbInitPickUpPark = false;
                        }
                    }
                    else if (wbInitHold){
                        if (gamepad1.x && gamepad1.left_bumper) {
                            //wobble move to end game drop position.
                            leftBumperOn = true;

                            //wobble arm move to hold position. Same position to drop the wobble
                            motorWBClaw.setTargetPosition(WOBBLEARM_ENDGAME + GlobalWobbleOffset);//(WOBBLEARM_AUTOLIFT);//(900);
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(0.6);//0.4);

                            //change status
                            wbHold = true;
                            wbInitHold = false;
                        }
//                        if (gamepad1.x) {
                        else if (gamepad1.x && !gamepad1.left_bumper) {
                            //Drop wobble from init position. Need more time compare to drop from end game position.
                            leftBumperOn = true;

                            // Stop motors when waiting for wobble move to position.
                            motorRightFront.setPower(0);
                            motorRightBack.setPower(0);
                            motorLeftFront.setPower(0);
                            motorLeftBack.setPower(0);

                            //wobble arm move to hold position. Same position to drop the wobble
                            motorWBClaw.setTargetPosition(WOBBLEARM_ENDGAME -200 + GlobalWobbleOffset);
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(0.9);//0.8);//1.0);//0.8);

                            //wait until reach end game drop position.
                            while(motorWBClaw.isBusy()){}
                            //after arm move to drop position then claw open
                            wobbleServo.setPosition(WOBBLECLAW_OPEN);
                            sleep(200);//300);//200);

                            //change status
                            wbDrop = true;
                            wbInitHold = false;
                        }
                        else if (gamepad1.a && !LeftTriggerOn && !leftBumperOn){
                            //in case pick up miss, move back to pick up position.
                            //wobble arm move to pick up position
                            motorWBClaw.setTargetPosition(WOBBLEARM_PICKUP + GlobalWobbleOffset); //800
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(1.0);//0.8);
                            //no waiting in maim loop
//                            while(motorWBClaw.isBusy()){}
                            //claw to open position
                            sleep(300);
                            wobbleServo.setPosition(WOBBLECLAW_OPEN);
                            sleep(100);//300);

                            //move block arms to pick up wobble position and they will not block shooting.
                            blockArmL.setPosition(BlockArmWbPickL);
                            blockArmR.setPosition(BlockArmWbPickR);
                            blockArmUp = true;

                            //change status
                            wbPickUp = true;
                            wbInitHold = false;
                        }
                        //alewady in init hold position
//                        else if (gamepad1.a && LeftTriggerOn) {
                            //in case pick up to end game position, move to init position.
//                            wobbleServo.setPosition(WOBBLECLAW_CLOSE);
//                            sleep(400);//300);
                            //wobble arm move to hold position. Same position to drop the wobble
//                            motorWBClaw.setTargetPosition(WOBBLEARM_INIT + GlobalWobbleOffset);//(WOBBLEARM_AUTOLIFT);//(900);
//                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            motorWBClaw.setPower(0.6);//0.4);

                            //no waiting in maim loop
//                            while(motorWBClaw.isBusy()){}
                            //change status
//                            wbInitHold = true;
//                            wbHold = true;
//                            wbInitHold = false;
//                        }

                    }

                    else if(wbHold){
                        if (gamepad1.a && !LeftTriggerOn && !leftBumperOn){
                            //move back to pick up position
                            //wobble arm move to pick up position
                            motorWBClaw.setTargetPosition(WOBBLEARM_PICKUP + GlobalWobbleOffset); //800
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(1.0);//0.8);
                            //no waiting in maim loop
//                            while(motorWBClaw.isBusy()){}
                            //claw to open position
                            sleep(300);
                            wobbleServo.setPosition(WOBBLECLAW_OPEN);
                            sleep(100);//300);

                            //move block arms to pick up wobble position and they will not block shooting.
                            blockArmL.setPosition(BlockArmWbPickL);
                            blockArmR.setPosition(BlockArmWbPickR);
                            blockArmUp = true;

                            //change status
                            wbPickUp = true;
                            wbHold = false;

                        }
                        else if (gamepad1.x && !LeftTriggerOn && !leftBumperOn){
                            // Stop bot first.
                            motorRightFront.setPower(0);
                            motorRightBack.setPower(0);
                            motorLeftFront.setPower(0);
                            motorLeftBack.setPower(0);
                            //after arm move to drop position, open claw.
                            wobbleServo.setPosition(WOBBLECLAW_OPEN);
                            sleep(200);//500);//400);//200);


                            wbDrop = true;
                            wbHold = false;
                        }
                        else if (gamepad1.a && LeftTriggerOn){//gamepad1.left_bumper) {
                            //wobble move to Init position in case accidently pick in hold position.
                            leftBumperOn = true;

                            //wobble arm move to hold position. Same position to drop the wobble
                            motorWBClaw.setTargetPosition(WOBBLEARM_INIT + GlobalWobbleOffset);//(WOBBLEARM_AUTOLIFT);//(900);
                            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            motorWBClaw.setPower(0.6);//0.4);

                            //change state
                            wbInitHold = true;
                            wbHold = false;
//                            wbPickUp = false;
                        }
                    }
                    else if(wbDrop){
                        //bot start move back until it is over write by stick control
                        while (    gamepad1.left_stick_x == 0
                                && gamepad1.left_stick_y == 0
                                && gamepad1.right_stick_x == 0){
                            WobbleDriveOn = true;
                            //drive bot back
                            motorRightFront.setPower(-0.6);//-0.7);//-0.5);
                            motorRightBack.setPower(-0.6);//-0.7);//-0.5);-0.5);
                            motorLeftFront.setPower(-0.6);//-0.7);//-0.5);-0.5);
                            motorLeftBack.setPower(-0.6);//-0.7);//-0.5);-0.5);
                        }

                        WobbleDriveOn = false;

                        //wobble arm move to tele park
                        motorWBClaw.setTargetPosition(WOBBLEARM_INIT + GlobalWobbleOffset);
                        motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorWBClaw.setPower(0.8);//

//                        sleep(100);
//                        wobbleServo.setPosition(WOBBLECLAW_PARK);
//                        sleep(500);//200);

                        wobbleServo.setPosition(WOBBLECLAW_PARK);

                        //no waiting in maim loop
//                        while(motorWBClaw.isBusy()){}

                        wbTelePark = true;
                        wbDrop = false;
                    }

            sleep(40);//50);//30);//10);
        }
        //Stop button pressed, need to stop all threads and motors.

//        wobbleCtrl.Stop();
//        wobbleCtrl.interrupt();
        //turn off all motors
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorShooterR.setPower(0);
        motorShooterL.setPower(0);
        motorIntake.setPower(0);
        motorWBClaw.setPower(0);

        //stop thread
        g_xy_Coordinate.StopGxy();
//        globalCoordinateUpdate.interrupt();
        g_xy_Coordinate.interrupt();
    }

    /**********************   WobbleClawCtrl Thread   ***********************************/


    public void turnCenter ( double speed,
                             double angle,
                             double driveTime,
                             double accuracy,      //accuracy in degree
                            int    brkOn,
                            int    brkOff)
    {
        double error= 0;;
        double steerTurn = 0;
        double timeDiff = 0;
        double currentTime= 0;;
        double prevTime= 0;;
        double errorSum = 0;
        boolean onTarget = false;
        double leftSpeed = 0;
        double rightSpeed = 0;
        double startTime= 0;;
        int cycleCnt = 0;
        int whileCnt = 0;
        int fineTurnCnt = 0;

        int brakeCnt = 0;
        int brakeFactor = 1;

        // Set motors mode. Can't not use encoder for drive train motors.
        // Otherwise will results in unexpected operations.
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //set drive train motor stop behavior to float.
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                (angleIMU.angleUnit, angleIMU.firstAngle));

        //calculate P
        double getPIDsteerP;

        if ( (Math.abs(angle - headingDoubleIMU) <= 3) ) {
            getPIDsteerP = 0.15;//0.16;
        }else if ( (Math.abs(angle - headingDoubleIMU) < 5) &&(Math.abs(angle - headingDoubleIMU) > 3) ) {
            getPIDsteerP = 0.12;//0.14;
        }else{
            getPIDsteerP = 0.1;
        }

        //calculate I
        double currentI = 0;
        double getPIDsteerI;
        if ( (Math.abs(angle - headingDoubleIMU) >= 10 ) ){
            getPIDsteerI = 0;
        }
        else {
            getPIDsteerI = 1;
        }

//        ElapsedTime holdTimer = new ElapsedTime();
        startTime = getRuntime();   //seconds
        prevTime = startTime;
        currentTime = startTime;


        boolean dirIndicator = true;
//        robotError = targetAngle - headingDoubleIMU;
//        while (robotError > 180) robotError -= 360;
//        while (robotError <= -180) robotError += 360;
        //right/left indicator
        if ((angle - headingDoubleIMU) > 0 && (angle - headingDoubleIMU) < 180){
            //left turn
            dirIndicator = false;
        }
        else if ((angle - headingDoubleIMU) > 180){
            //right turn
            dirIndicator = true;
        }
        else if ((angle - headingDoubleIMU) < 0 && (angle - headingDoubleIMU) > -180){
            //right turn
            dirIndicator = true;
        }
        else if ((angle - headingDoubleIMU) <= -180){
            //left turn
            dirIndicator = false;
        }

        double prevError = Math.abs(getAngleError(angle));
        while (opModeIsActive()
                && (!onTarget)
                && ((currentTime - startTime) < driveTime)
        )
        {
            whileCnt++;
            // Determine turn power based on Bot heading +/- error. Right shift +error, left shift -error
            error = getAngleError(angle);       // robotError = targetAngle - headingDoubleIMU;

            //Time difference
            currentTime = getRuntime();
            timeDiff = (currentTime - prevTime);
            //prevTime = currentTime;
            //errorSum += error * timeDiff;

            //ABS function
            if (Math.abs(error) > prevError){
                //When robot move away from target, enable ABS
                brakeCnt++;
//                if (rightSpeed < 0 && leftSpeed > 0) {
                if (brakeCnt <= brkOn) {
                    //set 0 power for motors braking
                    brakeFactor = 0;
                } else {
                    //to use normal PID control power
                    brakeFactor = 1;
                }
                if (brakeCnt == (brkOn + brkOff)) { brakeCnt = 0;}
            }
            else{
                brakeCnt = 0;
                brakeFactor = 1;
            }
            //save current error
            prevError = Math.abs(error);

 /**           //ABS function
            if (dirIndicator && (error > 3)){//3)){//2)){//3)){
                //right turn
                brakeCnt++;
//                if (rightSpeed < 0 && leftSpeed > 0) {
                if (brakeCnt <= brkOn) {
                    //set 0 power for motors braking
                    brakeFactor = 0;
                } else {
                    //to use normal PID control power
                    brakeFactor = 1;
                }
//                }
//                else{
//                    no brake for turn back to target
//                    brakeFactor = 1;
//                    brakeCnt = 0;
//                }
                if (brakeCnt == (brkOn + brkOff)) { brakeCnt = 0;}
            }
            else if (!dirIndicator && (error < -3)){//-3)){//-2)){//-3)) {
                //left turn
                brakeCnt++;
//                if (rightSpeed > 0 && leftSpeed < 0) {
                if (brakeCnt <= brkOn) {
                    //set 0 power for motors braking
                    brakeFactor = 0;
                } else {
                    //to use normal PID control power
                    brakeFactor = 1;
                }
//                }
//                else{
                //no brake for turn back to target
//                    brakeFactor = 1;
//                    brakeCnt = 0;
//                }

                if (brakeCnt == (brkOn + brkOff)) {brakeCnt = 0;}
            }
            else
            {
                brakeCnt = 0;
                brakeFactor = 1;
            }
*/
            // Bot heading is continuously in threshold 5 cycles. Turn will be done.
//            if (Math.abs(error) <= HEADING_THRESHOLD) {
            if (Math.abs(error) <= 3)
            {
                //only small move need to calculate I
                errorSum += error * timeDiff;
                if (Math.abs(error) <= accuracy)    //0.25)
                {
                    fineTurnCnt = 0;
                    cycleCnt++;
                    //                rightSpeed /= 2;
                    //                leftSpeed = -rightSpeed;
                    steerTurn = getPIDSteer(getPIDsteerP,
                            0.2,//0.25,
                            0.0001,//(0.0001)/2,   //(P_TURN_COEFF, 0.01, 0.00,
                            error, errorSum, timeDiff);

                    //steerTurn cliped in getPIDSteer()
                    //rightSpeed = Range.clip(speed * steerTurn, -1, 1);
                    rightSpeed = speed * steerTurn;// /2;
                    leftSpeed = -rightSpeed;
//                    steerTurn = 0.0;
//                    leftSpeed = 0.0;
//                    rightSpeed = 0.0;

                    if (cycleCnt > 4){//6){//5) { //about 150mS
                        steerTurn = 0.0;
                        leftSpeed = 0.0;
                        rightSpeed = 0.0;
                        onTarget = true;
                        //break; cycl
                    }
                }else {
                    //speed = speed/2;
                    fineTurnCnt++;
                    //currentI = getPIDsteerI*0.02;

                    if (fineTurnCnt <= 40){
                        currentI = 0.25;//getPIDsteerI*0.25;
                        //currentI = 0.02;
                    }
                    else if ( (fineTurnCnt > 40) && (fineTurnCnt <= 100)){
                        currentI = 0.2;//0.12;
                    } else if ( (fineTurnCnt > 100) && (fineTurnCnt <= 150)){
                        currentI = 0.15;
                    } else if (fineTurnCnt > 150){
                        currentI = 0.12;//0.2;
                    }

                    steerTurn = getPIDSteer(getPIDsteerP, currentI,
                            0.0001,//0.0, //(P_TURN_COEFF, 0.01, 0.00,
                            error, errorSum, timeDiff);


                    //steerTurn cliped in getPIDSteer()
                    //rightSpeed = Range.clip(speed * steerTurn, -1, 1);
                    rightSpeed = speed * steerTurn;// /2;
                    leftSpeed = -rightSpeed;
                    cycleCnt = 0;
                    onTarget = false;
                }
            }
            else{
                //Don't count large error move I
                errorSum = 0;
                steerTurn = getPIDSteer(getPIDsteerP, 0.0,
                        0.0001/2,//0.0001,//0.0,//(P_TURN_COEFF, 0.01, 0.00,
                        error, errorSum, timeDiff);

                //steerTurn cliped in getPIDSteer()
                //rightSpeed = Range.clip(speed * steerTurn, -1, 1);
                rightSpeed = speed * steerTurn;
                leftSpeed = -rightSpeed;
                cycleCnt = 0;
                fineTurnCnt = 0;
                onTarget = false;
            }

            // Send desired turn speeds to motors.
            motorRightFront.setPower(rightSpeed * brakeFactor);
            motorRightBack.setPower(rightSpeed * brakeFactor);
            motorLeftFront.setPower(leftSpeed * brakeFactor);
            motorLeftBack.setPower(leftSpeed * brakeFactor);

            // Display it for the driver.
            telemetry.addData("Cycle : whileCnt", "%5d : %5d", cycleCnt,whileCnt);
            telemetry.addData("Bot Heading: ", "%5.2f", (angle - error));
            telemetry.addData("Target angle", "%5.2f", angle);
            telemetry.addData("Err : St", "%5.2f : %5.2f", error, steerTurn);
            telemetry.addData("Speed.L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
            telemetry.addData("CurrentTime : PrevTime", "%5.2f : %5.2f", currentTime, prevTime);
            telemetry.addData("fineTurnCnt : timeDiff", "%5d : %5.2f", fineTurnCnt,timeDiff);
            telemetry.addData("Start : Elapse time: ", "%5.2f : %5.2f",startTime, (currentTime - startTime));
            telemetry.addData("onTarget : ", onTarget);
            telemetry.addData("opModeIsActive() : ", opModeIsActive());
            //telemetry.addData("End of ", "turnCenter().");
            //telemetry.addData("push gamepad1.y ", "to continue.");
            telemetry.update();
            prevTime = currentTime;

        }
        // Stop motors when exit.
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);

    }
    /**
     * Returns the robot's heading errors based on IMU.
     *
     * @return robotError
     */
    public double getAngleError ( double targetAngle){
        // Get robot heading from IMU

        double robotError;
        // calculate error in -179 to +180 range  (
        angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                (angleIMU.angleUnit, angleIMU.firstAngle));
        robotError = targetAngle - headingDoubleIMU;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        //sleep(30);
        return robotError;
    }


    public double getPIDSteer ( double P, double I, double D,
                                double error, double errorSum, double timeDiff){
        double driveSteer;
        driveSteer = P * error + I * errorSum + D * error / timeDiff;
        return Range.clip(driveSteer, -1, 1);
    }

    /*************************************************************************************
     * Global coordinate update Thread
     *
     *************************************************************************************/
    class G_xy_Coordinate extends Thread{
//    class G_xy_Coordinate implements Runnable{

        //Odometry wheels connected to Motor ports
//        DcMotorEx motorOdmR;    //Right Odometry at Hub 3 motor port 1
//        DcMotorEx motorOdmL;    //Left Odometry at Hub 3 motor port 3
//        DcMotorEx motorOdmB;    //Back Odometry at Hub

        //IMU for heading
//    BNO055IMU imu;

        private ElapsedTime runTime = new ElapsedTime();


        //Thread run condition
        private boolean isRunning = true;

        //Position variables used for storage and calculations, unit: tricks
        int odmTickR, odmTickL, prevOdmTickR, prevOdmTickL; //ticks counts
        int prevOdmTickCenterRL;
        int odmTickB, prevOdmTickB; //ticks counts

        //Global coordinate position in inch
        double globalPos_Gx = 0;
        double globalPos_Gy = 0;
        double globalAngle_theta = 0;
        double globalAngle_theta_degree = 0;
        int prevOdmYdiff;

        // Heading unit degree
//    double imuHeading;
//    double prevIMUHeading = 0;
//    double odmHeading = 0;

        double odmRL_Distance_inch = OdmRL_Distance_inch;    //inch
        double odmB_Distance_inch = OdmB_Distance_inch;
        int odmRL_Count_Per_inch = OdmRL_Count_Per_inch;;
        int odmB_Count_Per_inch = OdmB_Count_Per_inch;;
        //Update time interval (milliseconds) for the position update thread
        private int updateTime = UpdateTime;

        //Odometry tricks per cm.
        int odmRL_Count_Per_Cm, odmB_Count_Per_Cm;

        //Right-Left odometry distance in cm.
        double odmRL_Distance_Cm;
        //Back odometry distance in cm
        double odmB_Distance_Cm;

        int updateCycleCnt = 0;
        double elapsedTimeMax;
        double elapsedTimeMin;
        double elapsedTimeAvg;
        double prevTime = 0;
//        static double imuHeadingMain;

//        public G_xy_Coordinate(DcMotorEx motorOdmR,
//                               DcMotorEx motorOdmL,
//                               DcMotorEx motorOdmB,
//                               double imuHeadingMain,
//                               double odmRL_Distance_inch,    //inch
//                               double odmB_Distance_inch,
//                               int odmRL_Count_Per_inch,
//                               int odmB_Count_Per_inch,
//                               int updateTime)
//        {
//            this.motorOdmR = motorOdmR;
//            this.motorOdmL = motorOdmL;
//            this.motorOdmL = motorOdmB;
//            this.odmRL_Distance_inch = odmRL_Distance_inch;
//            this.odmB_Distance_inch = odmB_Distance_inch;
//            this.odmRL_Count_Per_inch = odmRL_Count_Per_inch;
//            this.odmB_Count_Per_inch = odmB_Count_Per_inch;
//            this.updateTime = updateTime;
//        }
        //Encoder_Odm_G_XY_Test d1 = new Encoder_Odm_G_XY_Test();

        /**
         * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
         */
        public void globalCoordinateUpdate() {
            //Get Current Positions, can be used for relative position move.
            odmTickR = -motorRightFront.getCurrentPosition();  //ticks
            odmTickL = -motorLeftFront.getCurrentPosition();
            odmTickB = -motorRightBack.getCurrentPosition();

            int odmTickChangeR = odmTickR -  prevOdmTickR;
            int odmTickChangeL = odmTickL -  prevOdmTickL;
            double odmPosChangeR = (double) odmTickChangeR / odmRL_Count_Per_inch;
            double odmPosChangeL = (double) odmTickChangeL / odmRL_Count_Per_inch;
//        int odmTickCenterRL = (int) ((odmTickR + odmTickL) / 2);
            int odmTickSumRL =  odmTickChangeR + odmTickChangeL;
            int odmTickDiffRL = odmTickChangeR - odmTickChangeL;
//        int odmTickCenterChangeRL = odmTickCenterRL - prevOdmTickCenterRL;
            int odmTickChangeB = odmTickB - prevOdmTickB;
            double odmPosChangeB = (double) odmTickChangeB / odmB_Count_Per_inch;

            //Heading angle change
            double delta_theta = (odmPosChangeR - odmPosChangeL) / (2 * odmRL_Distance_inch);

            //local pos change
            double rB = 0;
            double rRL = 0;
            double delta_X;
            double delta_Y;
//            if (delta_theta != 0){
//                rB = odmB_Distance_inch + odmPosChangeB / delta_theta;
//                rRL = (double)(odmRL_Distance_inch * (odmTickSumRL/odmTickDiffRL));
//            }
            if ((delta_theta == 0) || (odmTickDiffRL == 0)) {
                delta_X = (odmPosChangeR + odmPosChangeL) / 2;
                delta_Y = odmPosChangeB;
            }else{
//                rB =  odmPosChangeB / delta_theta + odmB_Distance_inch;
                rB =  odmPosChangeB / delta_theta - odmB_Distance_inch;
                rRL = (odmRL_Distance_inch * (double)(odmTickSumRL/odmTickDiffRL));
//                delta_X = rRL * Math.sin(delta_theta) - rB * (1 - Math.cos(delta_theta));
//                delta_Y = rRL * (1 - Math.cos(delta_theta)) + rB *  Math.sin(delta_theta);
                //based on <Mecanum Wheel Odometry>
                delta_X = rRL * (1 - Math.cos(delta_theta) - rB * Math.sin(delta_theta));
                delta_Y = rRL * Math.sin(delta_theta) + rB * (1 - Math.cos(delta_theta));
            }

            //global pos change
            double delta_Gx;
            double delta_Gy;
            delta_Gx = delta_X * Math.cos(globalAngle_theta) - delta_Y * Math.sin(globalAngle_theta);
            delta_Gy = delta_X * Math.sin(globalAngle_theta) + delta_Y * Math.cos(globalAngle_theta);

            //global pos coordinate
            globalPos_Gx += delta_Gx;       //inch
            globalPos_Gy += delta_Gy;       //inch
            globalAngle_theta += delta_theta;   //radius
            globalAngle_theta_degree = globalAngle_theta * 360 / (2 * PI);
            while (globalAngle_theta_degree > 180) globalAngle_theta_degree -= 360;
            while (globalAngle_theta_degree <= -180) globalAngle_theta_degree += 360;

            //save current ticks
            prevOdmTickR = odmTickR;
            prevOdmTickL = odmTickL;
            prevOdmTickB = odmTickB;

        }

        /**
         * Return robot heading based on odometry calculation, unit degree
         */
        public double getGlobalHeadingRadius(){ return (globalAngle_theta % (2 * Math.PI)); }
        public double getGlobalHeadingDegree(){ return (globalAngle_theta_degree );}// % 360); }

        /**
         * Returns the robot's right odometry current position
         * @return Odometry right position
         */
        public double getGlobalPos_X(){ return globalPos_Gx; }
        public double getGlobalPos_Y(){ return globalPos_Gy; }

        /**
         * Returns the robot's left odometry current position
         * @return Odometry lest position
         */
//    public int getOdmLPosition(){ return motorOdmLPosition; }


        /**
         * Reset global coordinate either based on odometry or IMU
         */
        public void resetGlobalCoordinate(){
            globalPos_Gx = 0;
            globalPos_Gy = 0;
            globalAngle_theta = 0;
            motorRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorRightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorOdmB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        public int getThreadOdmRTick(){ return odmTickR; }
        public int getThreadOdmLTick(){ return odmTickL; }
        public int getThreadOdmBTick(){ return odmTickB; }
        /**
         * Stops the position update thread
         */
        public void StopGxy(){ isRunning = false; }

        /**
         * Return thread run cycle
         */
        public int getUpdateCycle(){ return updateCycleCnt; }

        /**
         * Return thread average elapsed time interval.
         */
        public double getElapsedTimeAvg(){ return elapsedTimeAvg; }

        /**
         * Return thread maximum elapsed time interval.
         */
        public double getElapsedTimeMax(){ return elapsedTimeMax; }

        /**
         * Return thread minimum elapsed time interval.
         */
        public double getElapsedTimeMin(){ return elapsedTimeMin; }


        /**
         * Runs the thread
         */
        @Override
        public void run() {
            runTime.reset();
            updateCycleCnt = 0;
            int i = 0;
            int j = 0;
            double elapsedTimeAvgTemp = 0;

            while(isRunning) {
                updateCycleCnt++;

//            runTime.reset();
                double currentTime = runTime.seconds();
                double elapsedTime = currentTime - prevTime;
                prevTime = currentTime;

                if (j < 50){
                    j++;
                    if (elapsedTime > elapsedTimeMax && elapsedTime != 0)
                        elapsedTimeMax = elapsedTime;
                    if (elapsedTime < elapsedTimeMin && elapsedTime != 0)
                        elapsedTimeMin = elapsedTime;

                }
                else{
                    //Record 10 cycles max and min
                    elapsedTimeMax = elapsedTime;
                    elapsedTimeMin = elapsedTime;
                    j = 0;
                }

//            elapsedTimeAvg += elapsedTime;
//            elapsedTimeAvg /= updateCycleCnt;

                // Average 30 times. Fifth 10 cycle assign in else statement.
                if (i < 50){
                    i++;
                    elapsedTimeAvgTemp += elapsedTime;
                }
                else{
                    elapsedTimeAvg = elapsedTimeAvgTemp / 50;
                    elapsedTimeAvgTemp = elapsedTime;
                    i = 0;
                }

                globalCoordinateUpdate();

//            while (runTime.milliseconds() < updateTime){
//            }

//            while(elapsedTime < updateTime){}
//                try {
//                    Thread.sleep(30);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
            }
        }

    }

}
