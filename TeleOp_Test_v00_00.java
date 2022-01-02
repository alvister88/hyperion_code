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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 *    TeleOp_9614_B_v030_00
 *    . copy from TeleOp_9614_B_v020_02.
 *    . use control hub.
 *    . tested on 06/10
 *
 * =========== Configuration file testConfigHW =============
 *
 * motorOdmR and motorOdmF for verify encoder port purpose.
 * */

@TeleOp(name="TeleOp_Test_v00_00", group="TeleOp ...")
//@Disabled
public class TeleOp_Test_v00_00 extends LinearOpMode {
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

    //Shooting speed
    static final int ShooterSpeed = 2200;//2100;

    //global IMU angle offset in degree
//    static volatile double GlobalAngleOffSet = 0;
//    static volatile double GlobalLevelOffset = 0;
//    static volatile int GlobalWobbleOffset = 0;
    double GlobalAngleOffSet = 0;
    double GlobalLevelOffset = 0;
    int GlobalWobbleOffset = 0;


    static final double shootLevel_A = 0.52;//0.5;
    static final double shootLevel_B = 0.56;//0.55;
    static final double shootLevel_C = 0.6;
    static final double shootLevel_D = 0.65;    //for power shoot

    static final double shootAngle_A = -12.5;//-12;
    static final double shootAngle_B = -2.5;//-1;
    static final double shootAngle_C = 15;
    static final double shootAngle_D = 0;   //for power shoot

    //shooting servo
    static final double SHOOTPARK = 0.20;
    static final double SHOOTING = 0.39;

    //shooting levle servo
    static final double SHOOTLEVELPARK = 0.75;

    //Power shooting
    static final double shootPwAngle_1 = 3;
    static final double shootPwAngle_2 = 7.5;
    static final double shootPwAngle_3 = 13.5;

    static final double ShootPwLevel_1 = 0.55;
    static final double ShootPwLevel_2 = 0.55;
    static final double ShootPwLevel_3 = 0.55;
    //Block arm positions
    static final double BlockArmParkR = 0.12;
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

    int currentPositionIntake;
    boolean motorIntakeOnRdy = true;
    boolean motorIntakeOffRdy = false;


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

    double

    /**
     * Status machine
     */
    boolean wbTelePark = true;
    boolean wbPickUp   = false;
    boolean wbHold     = false;
    boolean wbDrop     = false;
    boolean wbInitHold = false;
    boolean wbInitPickUpPark = false;
    boolean wbInitPickUpEnd = false;

    double wbHoldStartTime = 0;
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
        motorRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

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
        motorRightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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


        //intake control, for contine 3 high goal shooting
        double motorIntakeOffStartTime = 0;

        //continue power shooting
        /**continue power shooting variables
         continuePwShootEn
         shootingPwRdy
         shootingPwCnt
         motorIntakeOffStartTime (global)
         **/
        boolean shootingPwRdy = false;
        boolean shootingPwRdy2 = false;
        int shootingPwCnt = 1;
        int shootingPwCnt2 = 1;
        boolean continuePwShootEn = true;
        boolean continuePwShootEn2 = true;

        //Single power shooting
//        boolean shootingPwCntEn = true;
//        boolean shootingPwCntEn2 = true;

        //1st position single power shooting
        boolean singlePwShootCntEn = true;
        int singlePwShootCnt = 1;
        //2nd position single power shooting
        boolean singlePwShootCntEn2 = true;
        int singlePwShootCnt2 = 1;

        /** single power shooting variables
         * shootingPwCntEn >> singlePwShootCntEn
         * shootingPwCnt >> singlePwShootCnt
         *
         local
         * pwShootCntEn >> singlePwShootCntEn
         * shootCntWait >> singlePwShootWait
         * singlePShootStartTime  >> singlePwShootStartTime
         *
         */
        boolean powerShootSingleEn = false;

        //For free single shooting.
        boolean shootingRdy = true;


        boolean circleDriveEn = false;
        int     circleDriveCnt = 0;

        boolean angleRightAdjRdy = true;
        boolean angleLeftAdjRdy = true;

        //PMT move
        boolean PMTmoveR = false;
        boolean PMTmoveL = false;

        //for continue 3 rings high goal shooting
        boolean highGoalSootEnd = false;

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        batteryVoltage = getBatteryVoltage();

        //move block arms to pick up wobble position and they will not block shooting.
        blockArmL.setPosition(BlockArmWbPickL);
//        blockArmR.setPosition(BlockArmWbPickR);
        blockArmUp = true;

        PIDFCoefficients pidOrig = motorShooterR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooterR.setVelocityPIDFCoefficients(NEW_P,NEW_I,NEW_D,NEW_F);
        motorShooterR.setPositionPIDFCoefficients(5.0);
//             re-read coefficients and verify change.
//            PIDFCoefficients pidModified = motorShooterR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        double precycleTime;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            LeftTriggerOn = Math.abs(gamepad1.left_trigger) > 0.5;
            RightTriggerOn = Math.abs(gamepad1.right_trigger) > 0.1;

            left_y = -gamepad1.left_stick_y;// / 3;    //reverse y direction for motor.
            left_x = gamepad1.left_stick_x;/// 3;    //make half power for testing.

            right_y = -gamepad1.right_stick_y;// / 3;    //reverse y direction for motor.

            if (!LeftTriggerOn) {
                //By pass for PMT move
                right_x = gamepad1.right_stick_x;/// 3;    //make half power for testing.
            }

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

        }

        /******************************************************************************************/
        //Stop button pressed, need to stop all motors and threads.
        //turn off all motors
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorShooterR.setPower(0);
        motorShooterL.setPower(0);
        motorIntake.setPower(0);
        motorWBClaw.setPower(0);
        shootServo.setPosition(SHOOTPARK);
    }

    /****************************** End Main Loop *************************************************/

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

}
