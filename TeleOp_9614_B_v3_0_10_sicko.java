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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

@TeleOp(name="TeleOp_9614_B_v3_0_10_sicko", group="TeleOp ...")
//@Disabled
public class TeleOp_9614_B_v3_0_10_sicko extends LinearOpMode {
    /* Declare OpMode members. */
    //motors
    DcMotorEx motorRightFront;  //odometry right side odmR
    DcMotorEx motorLeftFront;   //odometry left side odmL
    DcMotorEx motorRightBack;   //no encoder
    DcMotorEx motorLeftBack;    //no encoder
    DcMotorEx motorShooterR;
    DcMotorEx motorShooterL;
    DcMotorEx motorWBClaw;
    DcMotorEx motorIntake;      //no encoder

    //servos
    Servo wobbleServo;
    Servo shootLevelServo;
    Servo shootServo;
    Servo bumperServo;
    Servo blockArmL;

    //Gyro
    BNO055IMU imu;
    // IMU variables
    double headingDoubleIMU = 0;

    // global IMU angle offset in degree
    double GlobalAngleOffSet = 0;
    double GlobalLevelOffset = 0;
    int GlobalWobbleOffset = 0;

    // this stays because we use it (for some reason) when adjusting global offset.
    static final double shootLevel_A = 0.52;//0.5;

    //Block arm positions
//    static final double BlockArmParkL = 0.81;
    static final double BlockArmWbPickL = 0.81;

    public static final double NEW_P = 1.26;
    public static final double NEW_I = 0.126;
    public static final double NEW_D = 0.5;
    public static final double NEW_F = 12.6;

    double batteryVoltage;
    // Computes the current battery voltage
    double getBatteryVoltage() {
        double voltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = sensor.getVoltage();
        }
        return voltage;
    }

    boolean LeftTriggerOn = false;
    boolean leftBumperOn = false;
    boolean RightTriggerOn = false;

    boolean blockArmUp;

    BasicButtonWatcher x_watcher = new BasicButtonWatcher();
    BasicButtonWatcher dpad_up_watcher = new BasicButtonWatcher();
    BasicButtonWatcher dpad_down_watcher = new BasicButtonWatcher();
    BasicButtonWatcher dpad_left_watcher = new BasicButtonWatcher();
    BasicButtonWatcher dpad_right_watcher = new BasicButtonWatcher();
    BasicButtonWatcher back_watcher = new BasicButtonWatcher();
    BasicButtonWatcher right_stick_y_up_watcher = new BasicButtonWatcher();
    BasicButtonWatcher right_stick_y_down_watcher = new BasicButtonWatcher();

    ButtonWatcher y_watcher = new ButtonWatcher();

    void updateControls() {
        x_watcher.update(gamepad1.x);
        dpad_up_watcher.update(gamepad1.dpad_up);
        dpad_down_watcher.update(gamepad1.dpad_down);
        dpad_left_watcher.update(gamepad1.dpad_left);
        dpad_right_watcher.update(gamepad1.dpad_right);
        back_watcher.update(gamepad1.back);
        right_stick_y_up_watcher.update(gamepad1.right_stick_y < -0.5);
        right_stick_y_down_watcher.update(gamepad1.right_stick_y > 0.5);

        y_watcher.update(getRuntime(), gamepad1.y);
    }

    double left_y;
    double left_x;
    double right_y;
    double right_x;

    boolean motorShooterOn = false;
    boolean motorShooterOnRdy = true;

    boolean strafeForPosition2PowerShot = true;

    boolean motorIntakeOn = false;
    boolean reversingIntake = true;
    double reverseIntakeStartTime = -1;
    //intake control, for continuous 3 high goal shooting
    double intakeStartRequestTimestamp = -1;

    enum ClawState {TELEPARK, GRABBING_POSITION, HELD, JUST_DROPPED, INIT_HELD, REACHED_INTERMEDIATE_INIT_HOLD}
    ClawState wbClawState = ClawState.TELEPARK;

    double shootLevelCurrentPosition;

    enum TeamColor{RED, BLUE}
    TeamColor currentTeam;
    Params_Tele_Base params;

    void moveShootServo(int delay1, int delay2) {
        shootServo.setPosition(Params_Common.INDEXER_EXTENDED_VALUE);
        sleep(delay1);
        shootServo.setPosition(Params_Common.INDEXER_RETRACTED_VALUE);
        sleep(delay2);
    }

    void setDTMotors(double RF, double RB, double LF, double LB) {
        motorRightFront.setPower(RF);
        motorRightBack.setPower(RB);
        motorLeftFront.setPower(LF);
        motorLeftBack.setPower(LB);
    }

    void stopDTMotors() {
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
    }

    void startFlywheel(int adjustment) {
        motorShooterOn = true;
        motorShooterR.setVelocity(Params_Common.BASE_SHOOTER_RPM + adjustment - 250);
        motorShooterL.setVelocity(Params_Common.BASE_SHOOTER_RPM + adjustment - 250);
    }

    void stopFlywheel() {
        motorShooterOn = false;
        motorShooterR.setPower(0);
        motorShooterL.setPower(0);

        strafeForPosition2PowerShot = true;
    }

    void scheduleIntakeStart() {
        intakeStartRequestTimestamp = getRuntime();
    }

    void stopIntake() {
        motorIntake.setPower(0);
        motorIntakeOn = false;
        intakeStartRequestTimestamp = -1;
    }

    void moveWobbleArm(int position, double power) {
        motorWBClaw.setTargetPosition(position + GlobalWobbleOffset); //800
        motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorWBClaw.setPower(power);//1.0);
    }

    void wobbleDrive(double motorPower) {
        while (Math.abs(gamepad1.left_stick_x) < 0.1
                && Math.abs(gamepad1.left_stick_y) < 0.1
                && Math.abs(gamepad1.right_stick_x) < 0.1) {
            setDTMotors(motorPower, motorPower, motorPower, motorPower);
        }
        stopDTMotors();
    }

    // set Shoot Level - not affected by global offset.
    void setShootLevel_Fixed(double angle) {
        shootLevelCurrentPosition = angle;
        shootLevelServo.setPosition(angle);
    }

    void setShootLevel(double angle) {
        setShootLevel_Fixed(angle + GlobalLevelOffset);
    }

    void updateIMUVariables() {
        headingDoubleIMU = AngleUnit.DEGREES.normalize(
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
                        .firstAngle
        );
    }


    // Horribly Written Code
    // Sequence cannot be canceled.
    void strafeDist(int displacement_ticks) {
        // get sign; assume that displacement_ticks > 0
        int dir = Integer.signum(displacement_ticks);
        // left is positive, which is probably inconsistent
        int odmTickBMarker_temp = -motorRightBack.getCurrentPosition();
        double startTime = getRuntime();

        int dist = Math.abs(-motorRightBack.getCurrentPosition() - odmTickBMarker_temp);
        int targetdist = Math.abs(displacement_ticks);
        double v;

        while (dist < targetdist && getRuntime() - startTime < (double) 4) {
            dist = Math.abs(-motorRightBack.getCurrentPosition() - odmTickBMarker_temp);
            targetdist = Math.abs(displacement_ticks);
            v = (0.2 + (0.6 - 0.2) * Math.min(1.0, (targetdist - dist) / 250.0)) * dir;
            setDTMotors(v, -v, -v, v);
            if(getRuntime() - startTime > (double) 4) {
                break;
            }
        }
        stopDTMotors();
    }

    // this is the worst code I have ever written
    TeamColor getTeamInput() {
        while(!isStopRequested()) {
            if (gamepad1.x) { return TeamColor.BLUE; }
            if (gamepad1.b) { return TeamColor.RED; }
        }
        return TeamColor.BLUE;
    }


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
        bumperServo = hardwareMap.servo.get("blockArmR");

        //set motor directions
        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);  //odometry right side. forward --> negative counts
        motorRightBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorEx.Direction.FORWARD);   //odometry left side. forward --> negative counts
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
        updateIMUVariables();

        double mrfPower;
        double mrbPower;
        double mlfPower;
        double mlbPower;

        //set shooting servo to park position
        shootServo.setPosition(Params_Common.INDEXER_RETRACTED_VALUE);

        //set wobble claw to park position
        wobbleServo.setPosition(Params_Common.WB_CLAW_INIT);
        //initial shooting level set to position A
        setShootLevel(shootLevel_A);

        boolean continuePwShootEn = true;
        boolean continuePwShootEn2 = true;

        //1st position single power shooting
        boolean singlePwShootCntEn = true;
        int singlePwShootCnt = 0;
        //2nd position single power shooting
        boolean singlePwShootCntEn2 = true;
        int singlePwShootCnt2 = 0;

        //For free single shooting.
        boolean shootingRdy = true;

        //PMT move
        boolean PMTmoveR = false;
        boolean PMTmoveL = false;

        //for continue 3 rings high goal shooting
        boolean highGoalShootEnd = false;

        bumperServo.setPosition(Params_Common.BUMPER_SERVO_DOWN);
        ////////////////////////////////////////////////////////////////////////////

        double turnCoeff = 1.2;
        double shiftScale;

        currentTeam = getTeamInput();
        telemetry.addLine(currentTeam == TeamColor.BLUE ? "Team: Blue" : "Team: Red");
        telemetry.update();

        if (currentTeam == TeamColor.BLUE) {
            params = new Params_Tele_Blue();
        } else {
            params = new Params_Tele_Red();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        batteryVoltage = getBatteryVoltage();

        //move block arms to pick up wobble position and they will not block shooting.
        blockArmL.setPosition(BlockArmWbPickL);
        blockArmUp = true;

        //PIDFCoefficients pidOrig = motorShooterR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooterR.setVelocityPIDFCoefficients(NEW_P,NEW_I,NEW_D,NEW_F);
        motorShooterR.setPositionPIDFCoefficients(5.0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // 0.1 works fine here - used for shiftScale
            LeftTriggerOn = Math.abs(gamepad1.left_trigger) > 0.5;
            RightTriggerOn = Math.abs(gamepad1.right_trigger) > 0.1;

            updateControls();

            // left_y flipped so up corresponds w/ positive.
            left_y = -gamepad1.left_stick_y;
            left_x = gamepad1.left_stick_x;

            // right_y flipped so up corresponds w/ positive.
            right_y = -gamepad1.right_stick_y;
            right_x = gamepad1.right_stick_x;

            // sprint - power will be distributed based on translation and rotation and then scaled accordingly.
            // this means that turning is limited by movement speed, and vice versa.
            mlfPower = left_y+left_x + turnCoeff * right_x;
            mlbPower = left_y-left_x + turnCoeff * right_x;
            mrfPower = left_y-left_x - turnCoeff * right_x;
            mrbPower = left_y+left_x - turnCoeff * right_x;

            // ew
            if(RightTriggerOn) {
                shiftScale = 1.0-(gamepad1.right_trigger * 0.7);
                mlfPower *= shiftScale;
                mlbPower *= shiftScale;
                mrfPower *= shiftScale;
                mrbPower *= shiftScale;
            }

            if(Math.abs(mlfPower) > 1.0 || Math.abs(mlbPower) > 1.0 || Math.abs(mrfPower) > 1.0 || Math.abs(mrbPower) > 1.0) {
                double maxVal = Math.max(Math.max(Math.abs(mlfPower), Math.abs(mlbPower)), Math.max(Math.abs(mrfPower), Math.abs(mrbPower)));
                mlfPower/=maxVal;
                mlbPower/=maxVal;
                mrfPower/=maxVal;
                mrbPower/=maxVal;
            }

            if (!reversingIntake)     //skip left stick control and use intake reverse drive back control
            {
                //drive train motor power controlled by left stick
                setDTMotors(mrfPower, mrbPower, mlfPower, mlbPower);
            }

            //Manual single shooting. Need turn on shooter motor by press .b. No other control applied.
            if (gamepad1.right_bumper && motorShooterOn
                    && !LeftTriggerOn   //continue power shooting
                    && !leftBumperOn)   //single power shooting with position selectable
            {

                //Up to 4 rings shooting if hold gamepad1.right_bumper
                int shootFreeCnt = 1;
                while(gamepad1.right_bumper && motorShooterOn && shootFreeCnt <= 4 && shootingRdy){
                    moveShootServo(180, 180);
                    shootFreeCnt++;
                }
                shootingRdy = false;
            }
            else{
                shootingRdy = true;
            }

            // POSITION 1 CONTINUOUS POWER SHOOTING
            // NOTE:
            // For each position, all three power shot shots use the same level; this is set before the first shot.
            if (LeftTriggerOn){
                if (gamepad1.right_bumper && continuePwShootEn){
                    stopIntake();
                    startFlywheel(params.PS_SPEED_OFFSET_A);
                    setShootLevel_Fixed(params.PS_LEVEL_A);

                    //wait for flywheel to speed up
                    sleep(200);

                    if (gamepad1.right_bumper) {
                        turnCenter(0.55,
                                (params.PS_ANGLE_A1 + GlobalAngleOffSet),
                                5, 0.75, 5, 1);
                        moveShootServo(180,100);
                    }
                    if (gamepad1.right_bumper) {
                        turnCenter(0.55,
                                (params.PS_ANGLE_A2 + GlobalAngleOffSet),
                                5, 0.75, 5, 1);
                        moveShootServo(180,100);
                    }
                    if (gamepad1.right_bumper) {
                        turnCenter(0.55,
                                (params.PS_ANGLE_A3 + GlobalAngleOffSet),
                                5, 0.75, 5, 1);

                        moveShootServo(180, 0);
                    }

                    stopFlywheel();
                    scheduleIntakeStart();
                    continuePwShootEn = false;
                }
            }
            else{
                continuePwShootEn = true;
            }

            //2nd position continue power shooting.
            if (LeftTriggerOn){
                if (RightTriggerOn && continuePwShootEn2){
                    stopIntake();
                    startFlywheel(params.PS_SPEED_OFFSET_B);
                    setShootLevel_Fixed(params.PS_LEVEL_B);

                    if(strafeForPosition2PowerShot) {
                        strafeDist(params.SAFETY_STRAFE_DISPLACEMENT);
                        strafeForPosition2PowerShot = false;
                    }

                    //wait for flywheel to speed up
                    sleep(300);

                    if (gamepad1.right_trigger > 0.1) {
                        turnCenter(0.5,
                                (params.PS_ANGLE_B1 + GlobalAngleOffSet),
                                5, 0.5, 5, 1);
                        sleep(150);
                        moveShootServo(180,200);
                    }
                    if (gamepad1.right_trigger > 0.1) {
                        turnCenter(0.5,
                                (params.PS_ANGLE_B2 + GlobalAngleOffSet),
                                5, 0.5, 5, 1);
                        sleep(150);
                        moveShootServo(180,200);
                    }
                    if (gamepad1.right_trigger > 0.1) {
                        turnCenter(0.5,
                                (params.PS_ANGLE_B3 + GlobalAngleOffSet),
                                5, 0.5, 5, 1);
                        sleep(150);
                        moveShootServo(180,100);
                    }

                    stopFlywheel();
                    scheduleIntakeStart();

                    continuePwShootEn2 = false;
                }
            }
            else{
                continuePwShootEn2 = true;
            }

            //1st position single power shooting
            //Single power shooting. Shooting level will adjusted by GlobalAngleOffSet.
            //Select power shooting location. In case some power shooting missed in continuously shooting.
            //After adjust power shoot position, use single shooting control "right_bumper" to shoot.

            if (gamepad1.left_bumper){
                leftBumperOn = true;
                //one press increase 1 for counter
                if (gamepad1.right_bumper && singlePwShootCntEn){

                    double singlePwShootStartTime_L = getRuntime();
                    boolean singlePwShootCntEn_L = true;
                    boolean singlePwShootWait_L = true;

                    while( !((getRuntime() - singlePwShootStartTime_L) > 0.7) && singlePwShootWait_L && (singlePwShootCnt < 3)){
                        //singlePwShootCnt need to compatible with continue power shooting
                        if (gamepad1.right_bumper) {
                            if (singlePwShootCntEn_L) {
                                singlePwShootCnt++;
                                singlePwShootCntEn_L = false;
                            }
                        }
                        else{
                            singlePwShootCntEn_L = true;
                        }
                        //time out for each shoot selection
                        //exit if only one click in 200mS
                        if (((getRuntime() - singlePwShootStartTime_L) > 0.25) && (singlePwShootCnt == 1) ){
                            singlePwShootWait_L = false;
                        }
                        //exit if only two clicks in 500mS
                        if (((getRuntime() - singlePwShootStartTime_L) > 0.5) && (singlePwShootCnt == 2) ){
                            singlePwShootWait_L = false;
                        }
                    }
                    //turn off intake

                    //all 3 powershots use same level, which is affected by GlobalLevelOffset apparently
                    stopIntake();
                    startFlywheel(params.PS_SPEED_OFFSET_A);
                    setShootLevel(params.PS_LEVEL_A);

                    if(singlePwShootCnt == 1){
                        turnCenter(0.6,
                                (params.PS_ANGLE_A1 + GlobalAngleOffSet),
                                5, 1, 5, 1);
                    }
                    else if (singlePwShootCnt == 2){
                        turnCenter(0.5,
                                (params.PS_ANGLE_A2 + GlobalAngleOffSet),
                                5, 0.75, 5, 1);
                    }
                    else if (singlePwShootCnt == 3){
                        turnCenter(0.5,
                                (params.PS_ANGLE_A3 + GlobalAngleOffSet),
                                5, 0.75, 5, 1);
                    }
                    singlePwShootCntEn = false;
                }
            }
            else{
                leftBumperOn = false;
                singlePwShootCntEn = true;
                singlePwShootCnt = 0;
            }

            //2nd position single power shooting
            //Single power shooting. Shooting level will adjusted by GlobalAngleOffSet.
            //Select power shooting location. In case some power shooting missed in continuously shooting.
            //After adjust power shoot position, use single shooting control "right_bumper" to shoot.

            if (gamepad1.left_bumper){
                leftBumperOn = true;
                //one press increase 1 for counter
                if (RightTriggerOn && singlePwShootCntEn2){

                    double singlePwShootStartTime_L2 = getRuntime();
                    boolean singlePwShootCntEn_L2 = true;
                    boolean singlePwShootWait_L2 = true;

                    while( !((getRuntime() - singlePwShootStartTime_L2) > 0.9) && singlePwShootWait_L2 && (singlePwShootCnt2 < 3)){
                        RightTriggerOn = Math.abs(gamepad1.right_trigger) > 0.7;
                        if (RightTriggerOn) {
                            if (singlePwShootCntEn_L2) {
                                singlePwShootCnt2++;
                                singlePwShootCntEn_L2 = false;
                            }
                        }
                        else{
                            singlePwShootCntEn_L2 = true;
                        }
                        //time out for each shoot selection
                        //exit if only one click in 300mS
                        if (((getRuntime() - singlePwShootStartTime_L2) > 0.3) && (singlePwShootCnt2 == 1) ){
                            singlePwShootWait_L2 = false;
                        }
                        //exit if only two clicks in 600mS
                        if (((getRuntime() - singlePwShootStartTime_L2) > 0.6) && (singlePwShootCnt2 == 2) ){
                            singlePwShootWait_L2 = false;
                        }
                    }

                    stopIntake();
                    startFlywheel(params.PS_SPEED_OFFSET_B);
                    setShootLevel(params.PS_LEVEL_B);

                    if(strafeForPosition2PowerShot) {
                        strafeDist(params.SAFETY_STRAFE_DISPLACEMENT);
                        strafeForPosition2PowerShot = false;
                    }
                    sleep(300);

                    //turn robot
                    if(singlePwShootCnt2 == 1){
                        turnCenter(0.5,
                                (params.PS_ANGLE_B1 + GlobalAngleOffSet),
                                5, 1, 5, 1);
                    }
                    else if (singlePwShootCnt2 == 2){
                        turnCenter(0.5,
                                (params.PS_ANGLE_B2 + GlobalAngleOffSet),
                                5, 0.75, 5, 1);
                    }
                    else if (singlePwShootCnt2 == 3){
                        turnCenter(0.5,
                                (params.PS_ANGLE_B3 + GlobalAngleOffSet),
                                5, 0.75, 5, 1);
                    }
                    singlePwShootCntEn2 = false;
                }
            }
            else{
                leftBumperOn = false;
                singlePwShootCntEn2 = true;
                singlePwShootCnt2 = 0;
            }
            /////////////////////////////////////////////////////////////////////////////////////////////////////

            //forward/backward intake motor using right stick y
            if (!LeftTriggerOn) {  // since LT + right_y  also controls wobble adjust
                // START INTAKE
                if (gamepad1.right_stick_y < -0.5) {
                    motorIntake.setPower(1);
                    reverseIntakeStartTime = -1;
                    intakeStartRequestTimestamp = -1;
                    //for intake controlled by left bumper and double click right bumper
                    motorIntakeOn = true;
                }
                // STOP/REVERSE INTAKE
                else if (gamepad1.right_stick_y > 0.9) {
                    if (reverseIntakeStartTime == -1) {
                        reverseIntakeStartTime = getRuntime();
                    }
                    // Reverse Intake
                    if ((getRuntime() - reverseIntakeStartTime) > 0.25){
                        motorIntake.setPower(-1);
                        //drive bot back while drive on left stick
                        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                            setDTMotors(-0.4, -0.4, -0.4, -0.4);
                            reversingIntake = true; //only turn off motorIntake.setPower(-1) condition.
                        }
                        else{
                            reversingIntake = false;
                        }
                    } else {
                        stopIntake();
                    }
                }
                else {
                    reverseIntakeStartTime = -1;
                    if (reversingIntake) {
                        //it will not turn off intake when turn on by .b button
                        stopIntake();

                        //stop bot
                        //If left stick already take over control, skip set 0 power
                        if (gamepad1.left_stick_x != 0 && gamepad1.left_stick_y != 0) {
                            stopDTMotors();
                        }
                        //for intake controlled by button .b
                        reversingIntake = false;
                    }
                }
            }

            if (y_watcher.isJustHeld()) {
                shootServo.setPosition(Params_Common.INDEXER_FLUSH_VALUE);
                sleep(250);
                shootServo.setPosition(Params_Common.INDEXER_RETRACTED_VALUE);
            }

            if (gamepad1.b && !LeftTriggerOn && !leftBumperOn) {
                if (motorShooterOnRdy ) {
                    startFlywheel(-100);
                    stopIntake();
                }
                else {
                    stopFlywheel();
                    motorIntake.setPower(1);
                    motorIntakeOn = true;
                    intakeStartRequestTimestamp = -1;
                }
            }
            else{
                motorShooterOnRdy = !motorShooterOn;
            }

            if (back_watcher.isJustPressed()) {
                if (bumperServo.getPosition() == Params_Common.BUMPER_SERVO_DOWN) {
                    bumperServo.setPosition(Params_Common.BUMPER_SERVO_UP);
                } else {
                    bumperServo.setPosition(Params_Common.BUMPER_SERVO_DOWN);
                }
            }

            // RESET GYRO ANGLE
            if (gamepad1.y) {
                updateIMUVariables();
                GlobalAngleOffSet = headingDoubleIMU;
            }

            if (RightTriggerOn){
                ////////////////// POSITION A HIGH GOAL SHOOTING ///////////////////
                if(gamepad1.dpad_left && !highGoalShootEnd){
                    double motorShootStartTime = getRuntime();

                    startFlywheel(params.HIGH_GOAL_SPEED_OFFSET_A);
                    setShootLevel(params.HIGH_GOAL_LEVEL_A);

                    turnCenter(0.6,
                            (params.HIGH_GOAL_ANGLE_A + GlobalAngleOffSet),
                            5, 1.0, 5, 1);

                    // "wait" in case shooter flywheel has not reached the target speed yet
                    while ( ! ((getRuntime() - motorShootStartTime) > 0.3)) { if (isStopRequested()) break; }

                    // turn off intake
                    motorIntake.setPower(0);
                    motorIntakeOn = false;

                    // a bit more waiting (extra 0.1s)
                    while ( ! ((getRuntime() - motorShootStartTime) > 0.4)) { if (isStopRequested()) break; }

                    // fire 3 rings
                    // check dpad_left for each shot in case driver wants to stop shooting
                    if(gamepad1.dpad_left) { moveShootServo(150, 150); }
                    if(gamepad1.dpad_left) { moveShootServo(150, 150); }
                    if(gamepad1.dpad_left) { moveShootServo(150, 0); }

                    //Turn off shooter and schedule intake to be turned on
                    stopFlywheel();
                    scheduleIntakeStart();

                    highGoalShootEnd = true;
                }

                ////////////////// POSITION B HIGH GOAL SHOOTING ///////////////////
                else if (gamepad1.dpad_down && !highGoalShootEnd){

                    double motorShootStartTimeB = getRuntime();
                    //turn on shooter motors when start to continue shooting
                    startFlywheel(params.HIGH_GOAL_SPEED_OFFSET_B);
//                    }

                    //set shoot level
                    setShootLevel(params.HIGH_GOAL_LEVEL_B);
                    //turn bot to shoot angle
                    turnCenter(0.6,
                            (params.HIGH_GOAL_ANGLE_B + GlobalAngleOffSet),
                            5,
                            1.0,
                            5,//2,
                            1);
                    //wait for shooter motors stable
                    while ( ! ((getRuntime() - motorShootStartTimeB) > 0.3)) { if (isStopRequested()) break; }
                    //turn off intake
                    motorIntake.setPower(0);
                    motorIntakeOn = false;

                    while ( ! ((getRuntime() - motorShootStartTimeB) > 0.45)) { if (isStopRequested()) break; }
                    sleep(300); // klunky - gives driver time to cancel
                    if(gamepad1.dpad_down && motorShooterOn) {
                        //If gamepad1.dpad_left still hold on and shooter motors are on, then shooting
                        //continue 3 shoots
                        moveShootServo(150, 250);
                    }

                    //Can be stopped by release "gamepad1.dpad_left" between continue shootings to stop shooting.
                    if(gamepad1.dpad_down && motorShooterOn) {
                        //If gamepad1.dpad_left still hold on and shooter motors are on, then shooting
                        moveShootServo(150, 250);
                    }

                    //Can be stopped by released "gamepad1.dpad_left" between continue shootings to stop shooting.
                    if(gamepad1.dpad_down && motorShooterOn) {
                        //If gamepad1.dpad_left still hold on and shooter motors are on, then shooting
                        moveShootServo(150, 0);
                    }
                    //After shooting, turn off shooter and turn on intake(always ON).
                    stopFlywheel();
                    //delay to turn on intake
                    scheduleIntakeStart();
                    //exit position A continue high goal shooting
                    highGoalShootEnd = true;
                    //reset shooter motor turn on start time when finish shooting
//                    motorShootPrevTime = 0;
//                    motorShootStartTime = 0;
                }

                ////////////////// POSITION C HIGH GOAL SHOOTING ///////////////////
                else if (gamepad1.dpad_right && !highGoalShootEnd){
                    double motorShootStartTime = getRuntime();

                    // turn on shooter motors
                    startFlywheel(params.HIGH_GOAL_SPEED_OFFSET_C);

                    // set shooter level
                    // why does this not use globalangleoffset
                    setShootLevel(params.HIGH_GOAL_LEVEL_C);

                    stopDTMotors();
                    sleep(50);

                    if(gamepad1.dpad_right) { // f
                        // turn bot to shooting angle
                        turnCenter(0.5,
                                (params.HIGH_GOAL_ANGLE_C + GlobalAngleOffSet),
                                5, 1.0, 5, 1);

                        // turn off intake
                        motorIntake.setPower(0);
                        motorIntakeOn = false;

                        // a bit more waiting (extra 0.1s)
                        while ( ! ((getRuntime() - motorShootStartTime) > 0.3)) { if (isStopRequested()) break; }
                    }

                    // fire 3 rings
                    // check dpad_left for each shot in case driver wants to stop shooting
                    if(gamepad1.dpad_right) { moveShootServo(150, 150); }
                    if(gamepad1.dpad_right) { sleep(50); moveShootServo(150, 150); }
                    if(gamepad1.dpad_right) { sleep(50); moveShootServo(150, 0); }

                    //Turn off shooter and schedule intake to be turned on
                    stopFlywheel();
                    scheduleIntakeStart();

                    highGoalShootEnd = true;
                }

                ////////////////// POSITION D HIGH GOAL SHOOTING ///////////////////
                if(gamepad1.dpad_up && !highGoalShootEnd){
                    double motorShootStartTime = getRuntime();

                    // turn on shooter motors
                    startFlywheel(params.HIGH_GOAL_SPEED_OFFSET_D);

                    // set shooter level
                    setShootLevel(params.HIGH_GOAL_LEVEL_D);

                    // turn bot to shooting angle
                    turnCenter(0.6,
                            (params.HIGH_GOAL_ANGLE_D + GlobalAngleOffSet),
                            5, 1.0, 5, 1);

                    // "wait" in case shooter flywheel has not reached the target speed yet
                    while ( ! ((getRuntime() - motorShootStartTime) > 0.3)) { if (isStopRequested()) break; }

                    // turn off intake
                    motorIntake.setPower(0);
                    motorIntakeOn = false;

                    // a bit more waiting (extra 0.1s)
                    while ( ! ((getRuntime() - motorShootStartTime) > 0.4)) { if (isStopRequested()) break; }

                    // fire 3 rings
                    // check dpad_left for each shot in case driver wants to stop shooting
                    if(gamepad1.dpad_up) { moveShootServo(150, 150); }
                    if(gamepad1.dpad_up) { moveShootServo(150, 150); }
                    if(gamepad1.dpad_up) { moveShootServo(150, 0); }

                    //Turn off shooter and schedule intake to be turned on
                    stopFlywheel();
                    scheduleIntakeStart();

                    highGoalShootEnd = true;
                }
            }
            else {  //RightTriggerOn == false
                //reset continue 3 rings high goal shooting parameters.
                highGoalShootEnd = false;

                if (dpad_right_watcher.isJustPressed()) { GlobalAngleOffSet -= 2.5; }
                if (dpad_left_watcher.isJustPressed()) { GlobalAngleOffSet += 2.5; }

                //Manually adjust global shooting level and move level at same time for monitor.
                //All shooting position ABCD share same level offset
                if (dpad_up_watcher.isJustPressed()) {
                    if ((shootLevel_A + GlobalLevelOffset) >= 0.1) {
                        GlobalLevelOffset -= 0.02;//0.01;//0.025;
                        shootLevelCurrentPosition -= 0.02;//0.01;//0.025;
                    }
                    shootLevelServo.setPosition(shootLevelCurrentPosition);
                }

                //adjust shooting level down
                if (dpad_down_watcher.isJustPressed()) {
                    if ((shootLevel_A + GlobalLevelOffset) <= 0.75) {
                        GlobalLevelOffset += 0.02;
                        shootLevelCurrentPosition += 0.02;
                    }
                    shootLevelServo.setPosition(shootLevelCurrentPosition);
                }
            }

            //turn on intake after continue 3 rings high goal and power shooting
            if (!motorIntakeOn && intakeStartRequestTimestamp != -1) {
                if ((getRuntime() - intakeStartRequestTimestamp) > 1.2){// 0.8) {
                    motorIntake.setPower(1);
                    intakeStartRequestTimestamp = -1;
                    motorIntakeOn = true;
                }
            }

            //set bot to current 0 degree reference
            if (gamepad1.b && !LeftTriggerOn && leftBumperOn) {
                turnCenter(0.5,
                        (0 + GlobalAngleOffSet ),
                        3, 0.5, 2, 1);
            }

            // micro turns - used when adjusting angle for power shots.
            // also adjusts the globalAngleOffset (even though we usually reset the offset right after)
            if (gamepad1.b && LeftTriggerOn && !leftBumperOn) {
                stopIntake();
                updateIMUVariables();
                double GlobalAngleOffSetTemp = headingDoubleIMU;

                setDTMotors(-0.2, -0.2, 0.2, 0.2);
                sleep(50);
                stopDTMotors();
                sleep(150);

                updateIMUVariables();
                GlobalAngleOffSet += headingDoubleIMU - GlobalAngleOffSetTemp;
            }
            if (gamepad1.x && LeftTriggerOn) {
                stopIntake();
                updateIMUVariables();
                double GlobalAngleOffSetTemp = headingDoubleIMU;

                setDTMotors(0.2, 0.2, -0.2, -0.2);
                sleep(50);
                stopDTMotors();
                sleep(150);

                updateIMUVariables();
                GlobalAngleOffSet += headingDoubleIMU - GlobalAngleOffSetTemp;
            }

        ////////////////////////////////////////////////////////////////////////////////////////////
            // Wobble Arm Manual Adjustment
            if (LeftTriggerOn){
                if (right_stick_y_down_watcher.isJustPressed()) {
                    GlobalWobbleOffset -= 50;

                    //move wobble arm to adjusted position
                    motorWBClaw.setTargetPosition(motorWBClaw.getCurrentPosition() - 50);
                    motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorWBClaw.setPower(1.0);
                }

                if (right_stick_y_up_watcher.isJustPressed()) {
                    GlobalWobbleOffset += 50;

                    //move wobble arm to adjusted position
                    motorWBClaw.setTargetPosition(motorWBClaw.getCurrentPosition() + 50);
                    motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorWBClaw.setPower(1.0);
                }
            }
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
            //Wobble Arm State Machine
            switch (wbClawState) {
                case TELEPARK: {
                    if (gamepad1.a && !LeftTriggerOn){
                        // MOVE WOBBLE ARM TO GRABBING POSITION
                        moveWobbleArm(Params_Common.WB_ARM_GRAB, 1.0);
                        sleep(200); // why is there a sleep here
                        wobbleServo.setPosition(Params_Common.WB_CLAW_OPEN);

                        wbClawState = ClawState.GRABBING_POSITION;
                    }
                    break;
                }
                case GRABBING_POSITION: {
                    if (!motorWBClaw.isBusy()) { // wait for arm to finish moving
                        if (gamepad1.a) {
                            if (LeftTriggerOn) {
                                // GRAB AND STORE WOBBLE IN INIT POSITION
                                stopDTMotors();
                                wobbleServo.setPosition(Params_Common.WB_CLAW_CLOSE);
                                sleep(300);

                                // hover grab
                                moveWobbleArm(Params_Common.WB_ARM_GRAB_HOVER, 0.8);
                                wobbleDrive(-0.7);
                                moveWobbleArm(Params_Common.WB_ARM_INIT_INTERMEDIATE, 0.9);

                                wbClawState = ClawState.REACHED_INTERMEDIATE_INIT_HOLD;
                            } else {
                                // GRAB WOBBLE AND MOVE TO DROPPING POSITION
                                stopDTMotors();
                                wobbleServo.setPosition(Params_Common.WB_CLAW_CLOSE);
                                sleep(400);

                                // hover grab
                                moveWobbleArm(Params_Common.WB_ARM_GRAB_HOVER, 0.8);
                                wobbleDrive(-0.7);
                                moveWobbleArm(Params_Common.WB_ARM_DROP, 0.8);

                                wbClawState = ClawState.HELD;
                            }
                        }
                    }
                    break;
                }
                case REACHED_INTERMEDIATE_INIT_HOLD: {
                    // MOVE FROM INTERMEDIATE INIT TO INIT
                    if (!motorWBClaw.isBusy()) { // wait for arm to finish moving
                        moveWobbleArm(Params_Common.WB_ARM_INIT, 0.3);

                        wbClawState = ClawState.INIT_HELD;
                    }
                    break;
                }
                case INIT_HELD: {
                    if (!LeftTriggerOn && x_watcher.isJustPressed()) {
                        // MOVE FROM INIT TO DROPPING POSITION
                        leftBumperOn = true;

                        moveWobbleArm(Params_Common.WB_ARM_DROP, 0.6);

                        wbClawState = ClawState.HELD;
                    } else if (gamepad1.a && !LeftTriggerOn) {
                        if (leftBumperOn) {
                            // LOOSEN CLAW TO ALLOW WOBBLE TO DROP DOWN IN INIT POSITION
                            wobbleServo.setPosition(Params_Common.WB_CLAW_LOOSEN);
                            sleep(100);
                            wobbleServo.setPosition(Params_Common.WB_CLAW_CLOSE);
                        } else {
                            // MOVE TO GRABBING POSITION
                            moveWobbleArm(Params_Common.WB_ARM_GRAB, 0.8);
                            sleep(300);
                            wobbleServo.setPosition(Params_Common.WB_CLAW_OPEN);
                            sleep(100);

                            wbClawState = ClawState.GRABBING_POSITION;
                        }
                    }
                    break;
                }
                case HELD: {
                    bumperServo.setPosition(Params_Common.BUMPER_SERVO_UP);
                    if (gamepad1.a && !LeftTriggerOn && !leftBumperOn){
                        // MOVE FROM DROP POSITION TO GRAB POSITION AND LET GO
                        moveWobbleArm(Params_Common.WB_ARM_GRAB, 1.0);

                        sleep(300);
                        wobbleServo.setPosition(Params_Common.WB_CLAW_OPEN);
                        sleep(100);

                        bumperServo.setPosition(Params_Common.BUMPER_SERVO_DOWN);

                        wbClawState = ClawState.GRABBING_POSITION;
                    }
                    else if (x_watcher.isJustPressed() && !LeftTriggerOn && !leftBumperOn){
                        // DROP WOBBLE
                        stopDTMotors();
                        wobbleServo.setPosition(Params_Common.WB_CLAW_OPEN);
                        sleep(200);

                        wbClawState = ClawState.JUST_DROPPED;
                    }
                    else if (gamepad1.a && LeftTriggerOn){
                        // MOVE FROM DROP POSITION TO INIT POSITION
                        leftBumperOn = true;

                        moveWobbleArm(Params_Common.WB_ARM_INIT, 0.6);

                        wbClawState = ClawState.INIT_HELD;
                    }
                    break;
                }
                case JUST_DROPPED: {
                    // AFTER DROPPING - RESET TO INIT POSITION
                    wobbleDrive(-0.6);

                    bumperServo.setPosition(Params_Common.BUMPER_SERVO_DOWN);

                    moveWobbleArm(Params_Common.WB_ARM_INIT, 0.8);
                    wobbleServo.setPosition(Params_Common.WB_CLAW_INIT);

                    wbClawState = ClawState.TELEPARK;
                    break;
                }
                default: break;
            }
            sleep(20);
        }

        //Stop button pressed, need to stop all motors.
        stopDTMotors();
        stopFlywheel();
        motorIntake.setPower(0);
        motorWBClaw.setPower(0);
        shootServo.setPosition(Params_Common.INDEXER_RETRACTED_VALUE);
    }

    // END MAIN LOOP /////////////////////////////////////////////////////

    public void turnCenter ( double speed,
                             double angle,
                             double timeout, // ? hard time limit; deadline
                             double accuracy,      //accuracy in degree
                             int    abs_BrakeDuration,
                             int    abs_NoBrakeDuration)
    {
        double var_P;
        double var_I;
        double var_D;

        double error;
        double absError;
        double prevAbsError;

        double turnSpeed;

        double startTime = getRuntime();
        double currentTime = startTime;
        double prevTime = startTime;

        double timeDiff;
        double errorSum = 0;

        int debug_loopCycleCount = 0;
        int fineTurnCnt = 0;
        int onTargetCnt = 0;

        int brakeCnt = 0;
        int brakeFactor;

        // Set motors mode. Cannot use encoder for drive train motors.
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

        // calculate var_P, which remains constant throughout the while loop
        // the initial error is used to calculate P; this error also gets stored into prevAbsError.
        updateIMUVariables();
        prevAbsError = Math.abs(getAngleError(angle));

        if ( (prevAbsError <= 3) ) { var_P = 0.15; }
        else if (prevAbsError < 5) { var_P = 0.12; }
        else { var_P = 0.1; }

        // Main loop
        // var_I and var_D vary based on how close the robot is to the target
        while (opModeIsActive() && ((currentTime - startTime) < timeout)) {
            debug_loopCycleCount++;

            // Calculate Error - Right shift is positive, left shift is negative.
            error = getAngleError(angle);
            absError = Math.abs(error);

            // Time difference
            currentTime = getRuntime();
            timeDiff = (currentTime - prevTime);

            // ABS - Anti-lock braking system if robot is moving away from target.
            // I don't know why, but the pid doesn't work without it
            if (absError > prevAbsError){
                if (brakeCnt % (abs_BrakeDuration + abs_NoBrakeDuration) <= abs_BrakeDuration) {
                    brakeFactor = 0; //brake - set motor power to 0x
                } else {
                    brakeFactor = 1; //no brake
                }
                brakeCnt++;
            }
            else {
                brakeCnt = 0;
                brakeFactor = 1;
            }

            if (absError > 3) {
                // when angle is way off - ignore I in PID calculation.
                errorSum = 0;
                onTargetCnt = 0;
                fineTurnCnt = 0;

                // I and errorSum both stay at 0.
                var_I = 0;
                var_D = 0.0005;
            } else {
                errorSum += error * timeDiff;
                if (absError < accuracy) {
                    // when on target - turning is finished if on target for >4 cycles.
                    fineTurnCnt = 0;
                    onTargetCnt ++;

                    var_I = 0.2;

                    // PID TURNING COMPLETE
                    if (onTargetCnt > 6) break;
                } else {
                    // when not on target but within 3 deg - var_I varies w/ time
                    onTargetCnt = 0;
                    fineTurnCnt ++;
                    if (fineTurnCnt < 40){ var_I = 0.25; }
                    else if (fineTurnCnt < 100) { var_I = 0.2; }
                    else if (fineTurnCnt < 150) { var_I = 0.15; }
                    else { var_I = 0.12; }
                }
                var_D = 0.0001;
            }

            turnSpeed = getPIDSteer(var_P, var_I, var_D, error, errorSum, timeDiff);
            turnSpeed *= speed * brakeFactor;

            setDTMotors(turnSpeed, turnSpeed, -turnSpeed, -turnSpeed);

            // Display it for the driver.
            telemetry.addData("Claw value : ", "%5.2f", wobbleServo.getPosition());
            telemetry.addData("Cycle : debug_loopCycleCount", "%5d : %5d", onTargetCnt,debug_loopCycleCount);
            telemetry.addData("Bot Heading: ", "%5.2f", (angle - error));
            telemetry.addData("Target angle", "%5.2f", angle);
            telemetry.addData("Err : St", "%5.2f : %5.2f", error, turnSpeed);
            telemetry.addData("CurrentTime : PrevTime", "%5.2f : %5.2f", currentTime, prevTime);
            telemetry.addData("fineTurnCnt : timeDiff", "%5d : %5.2f", fineTurnCnt, timeDiff);
            telemetry.addData("Start : Elapse time: ", "%5.2f : %5.2f",startTime, (currentTime - startTime));
            telemetry.addData("opModeIsActive() : ", opModeIsActive());
            telemetry.update();

            // update variables
            prevAbsError = absError;
            prevTime = currentTime;
        }
        // Stop motors when exit.
        stopDTMotors();
    }

    public double getAngleError ( double targetAngle ) {
        // Get robot heading from IMU
        updateIMUVariables();
        return AngleUnit.DEGREES.normalize(targetAngle - headingDoubleIMU);
    }

    // our pid calculation (D) is wrong
    public double getPIDSteer ( double P, double I, double D, double error, double errorSum, double timeDiff ) {
        double driveSteer = P * error + I * errorSum + D * error / timeDiff;
        return Range.clip(driveSteer, -1, 1);
    }
}