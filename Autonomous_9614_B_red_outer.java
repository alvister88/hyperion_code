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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
/**

 ----------------------------------------------------------------------
 v80_00
 . Change to use control hub and web camera


 * =========== Configuration file testConfigHW =============
 *
 * motorOdmR and motorOdmF for verify encoder port purpose.
 * */

/**
 * =========== Configuration file initTest =============
 */

@Autonomous(name="Autonomous_9614_B_red_outer", group="Autonomous9614")
//@Disabled
public class Autonomous_9614_B_red_outer extends LinearOpMode {

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
    Servo bumperServo;
    Servo blockArmL;

    //    VoltageSensor voltSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
    //Gyro
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
//    Acceleration gravity;

    /**======================================================================================
     * GLOBAL VARIABLE ADJUSTMENT
     * =====================================================================================*/
//    double GbAngleAdj = 0;
//    double GbLevelAgj = 0;
    static final double GbAngleAdj = 0;
    static final double GbLevelAgj = 0;
    /** =====================================================================================*/

    //Image recognition initial
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            " AVfxzRP/////AAABmSIhZdysSEYht/BcveKldqY59FlBYRWfPJvW2T8vi49/b9KJcMbScVAiGHaiOJWsNSlwuRVYZc7zH5jtBOsiSN4LrNq3bcjx5Vu2UUpRCYuooTMJVY07A9qLWMvd9eWgklXm2vBCG9OgbzXHztc0SoFurqV2Yq0Fu9/PbslqOeKk2eMEo1l007NTxiihKg4iQXtBPVfPF+dsykiBWn0RVX2Ut4a7zeqV1LZwfP8nKepNv7hqkfEsnPwhTOxQyJz7fZEVtGdUMhY7ZldwVpJtCpY0vIpbgxXSXTJH1COEJnP46mcNjlyTgSTLutB12b+aJHtiM2mhE30ijWOJUZ2OEP9liKk9saTxA1/kgZPFFi8L";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    static final double COUNTS_PER_ODM_REV = 1200; // Odometry counts 300x4
    //static final double COUNTS_PER_MOTOR_REV = 538;// neverRest 20:1//1120; // motor
    //static final double COUNTS_PER_MOTOR_REV = 538;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double ODM_DIAMETER_INCHES = 2.25;     // For figuring circumference
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final int ODM_COUNTS_PER_INCH = (int) ((COUNTS_PER_ODM_REV * DRIVE_GEAR_REDUCTION) /
            (ODM_DIAMETER_INCHES * 3.1415));  // = 169.77
    static final double WHEEL_ODM_RATIO = WHEEL_DIAMETER_INCHES / ODM_DIAMETER_INCHES;
    static final double ODM_DISTANCE_RL = 13.5;      //inch

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.
    //static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    //static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    //Robot turn PID coefficients
    static final double HEADING_THRESHOLD = 0.5;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double I_TURN_COEFF = 0;
    static final double D_TURN_COEFF = 0;

    //shooting servo
    static final double SHOOTPARK = 0.2;//0.18;
    static final double SHOOTING = 0.45;

    static final double BUMPER_SERVO_DOWN = 0.20;
    static final double BUMPER_SERVO_UP = 0.57;

    //shooting servo
    static final double SHOOTLEVELPARK = 0.75;

    //wobble claw servo
//    static final double WOBBLECLAW_OPEN = 1;//0.95;
//    static final double WOBBLECLAW_CLOSE = 0.48;
//    static final double WOBBLECLAW_PARK = 0.55;
    //wobble claw servo
    static final double WOBBLECLAW_OPEN = 0.90;//0.95;
    static final double WOBBLECLAW_CLOSE = 0.30;//0.33;
    static final double WOBBLECLAW_PARK = 0.29;//0.30;


    //public double drivePowerWeight() variables
    static final double SHORT_DIS_POWER = 0.5;  //used for small distance move
    static final double START_POWER = 0.3; //drive train start power
    static final double SET_POWER = 1.0;//0.7;  //setPower;
    static final double STOP_POWER = 0.3;   //drive train stop power
    static final int SET_POINT = 1200 /3 ;// 2;  //half turn of odometry wheel
    static final int DECREASE_POINT = 1200 *3/2 ;//* 2; // 3 turn of wheel
    static final int STOP_RANGE = 1200;// * 3 / 2; // 3/2 turn of wheel

    //Parameters used for Y direction fine tune PID
    static final double P_Y_COEFF = 0.02 / 20;
    static final double I_Y_COEFF = 0.0;
    static final double D_Y_COEFF = 0;


    //wobble arm for 60:1 motor
    static final int WOBBLEARM_INIT     = 0;    // wobble teleOp park
    //    static final int WOBBLEARM_HORIZON = -80;
    static final int WOBBLEARM_ENDGAME = -840;//-830;//800;//-775;//-750;//-500;
    static final int WOBBLEARM_AUTOLIFT = -1600;//-1550;//-1600;
    static final int WOBBLEARM_PICKUP = -1675;//-1700;//-1750;

    //new wobble arm for 40:1 motor
//    static final int WOBBLEARM_INIT     = 0;
//    static final int WOBBLEARM_HORIZON = -80;
//    static final int WOBBLEARM_ENDGAME = -580;
//    static final int WOBBLEARM_AUTOLIFT = -1025;
//    static final int WOBBLEARM_PICKUP = -1120;
    /**
     static final int WOBBLEARM_INIT = 0;
     static final int WOBBLEARM_HOLD = 2500;//2200;//2700;
     static final int WOBBLEARM_INITLEFT = 2500;//2200;//2700;
     static final int WOBBLEARM_PICKUP = 750;
     static final int WOBBLEARM_AUTOLIFT = 900;//1000;
     static final int WOBBLEARM_TELEPARK = 2500;//2400;//2300;
     static final int WOBBLEARM_ENDLIFT = 2000;
     */

    //Block arms initial position
    static final double BlockArmShootR = 0.4;
    static final double BlockArmShootL = 0.81;

    static final double BlockArmParkR = 0.23;
    static final double BlockArmParkL = 0.81;


    static volatile boolean driveTrainRdy = true;
    static volatile boolean wobbleClawRdy = false;
    static volatile boolean shooterRdy = false;

    //To handle Zone B end no movement
    static volatile boolean wobbleArmEnd = false;

    static volatile int configSetup;// = 3;//1;//2;        //initial value
    double recogRingHeight;
    double recogRingAspectRatio;

    // Global variables for angle based on IUM.
    double headingDouble = 0;

    double batteryVoltage;
    double batteryVoltageClass;

    //Shooting motor
    static final int MotorSpeedSet = 1700;// 2100;


    HardwareMap.DeviceMapping<VoltageSensor> vSensor;

//    VoltageSensor vSensor;
//    batteryVoltageClass = vSensor.getVoltage();

    // Computes the current battery voltage
    double getBatteryVoltage() {
        double voltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = sensor.getVoltage();
        }
        return voltage;
    }

    void moveShootServo(int delay1, int delay2) {
        shootServo.setPosition(SHOOTING);
        sleep(delay1);
        shootServo.setPosition(SHOOTPARK);
        sleep(delay2);
    }

    // temporary; might get rid of this completely
    void startFlywheel(int speed) {
        motorShooterR.setVelocity(speed);
        motorShooterL.setVelocity(speed);
    }

    void stopFlywheel() {
        motorShooterR.setVelocity(0);
        motorShooterL.setVelocity(0);
    }

    //shooter control thread
    ShootingCtrl shootingCtrl = new ShootingCtrl();//driveTrainRdy, wobbleClawRdy, shooterRdy,

    //wobble control thread
//    WobbleClawCtrl wobbleClawCtrl = new WobbleClawCtrl();//(driveTrainRdy, wobbleClawRdy, shooterRdy,


    @Override
    public void runOpMode() {

        //Thread messages
        //boolean driveTrainRdy,wobbleClawRdy,shooterRdy;

        /*
         * Initialize hardware.
         */
        //motors
        motorRightFront = hardwareMap.get(DcMotorEx.class, "MRF");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "MRB");
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "MLF");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "MLB");
        motorShooterR = hardwareMap.get(DcMotorEx.class, "ShootR");
        motorShooterL = hardwareMap.get(DcMotorEx.class, "ShootL");
        motorWBClaw = hardwareMap.get(DcMotorEx.class, "WobbleClaw");
        motorIntake = hardwareMap.get(DcMotorEx.class, "Intake");

        //servos
        shootServo = hardwareMap.servo.get("shootServo");
        wobbleServo = hardwareMap.servo.get("WBGriper");
        shootLevelServo = hardwareMap.servo.get("ShootLevel");

        blockArmL = hardwareMap.servo.get("blockArmL");
        bumperServo = hardwareMap.servo.get("blockArmR");

//            HardwareMap.DeviceMapping<VoltageSensor> voltageSensor;
        vSensor = hardwareMap.voltageSensor;

        //set motors direction
        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);  //odomety right side. Move forward odometry get negative counts
        motorRightBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorEx.Direction.FORWARD);   //odometry left side. Move forward odometry get negative counts
        motorLeftBack.setDirection(DcMotorEx.Direction.FORWARD);
        motorShooterR.setDirection(DcMotorEx.Direction.REVERSE);
        motorShooterL.setDirection(DcMotorEx.Direction.REVERSE);
        motorWBClaw.setDirection(DcMotorEx.Direction.FORWARD);
        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);

        motorRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  //odomety right side
        //motorRightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);   //odometry left side
        //motorLeftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorShooterR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorShooterL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorWBClaw.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //motorIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motorShooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorShooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorWBClaw.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //set shooting servo to park position
        shootServo.setPosition(SHOOTPARK);

        // set shooting level servo to park position
        shootLevelServo.setPosition(SHOOTLEVELPARK);

        bumperServo.setPosition(BUMPER_SERVO_UP);

        //leave wobble servo float
//        wobbleServo.setPosition(WOBBLECLAW_PARK);

        //Block arms  initial to park position. Block arms will not use in autonomous.
//        blockArmL.setPosition(BlockArmParkL);
//        blockArmR.setPosition(BlockArmParkR);

        //place wobble claw arm at initial position before load the code
        //gripe the wobble and move arm to auto high hold position

//        while (!gamepad1.y) {
//            telemetry.addData("1. Set Wobble Arm to ", " Autonomous Init Position.");
//            telemetry.addData("2. Push Gamepad_1.Y", " to continue...");
//            telemetry.update();
//        }

        //gripe wobble
        //Manually set wobble arm positino to initial position. It is autonomous wobble hold position.
        motorWBClaw.setTargetPosition(WOBBLEARM_INIT);
        motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorWBClaw.setPower(0.2);
        while (motorWBClaw.isBusy()) {
//            set claw to open position while arm moving??claw will hit bot
            wobbleServo.setPosition(WOBBLECLAW_OPEN);
            telemetry.addData("Claw moving.", "Wait.. ");
            telemetry.update();
        }

        //set claw to open position
        wobbleServo.setPosition(WOBBLECLAW_OPEN);
        telemetry.addData("Put wobble into claw, hold and  ", "wait until it close.. ");
        telemetry.update();
        sleep(2500);
        wobbleServo.setPosition(WOBBLECLAW_CLOSE);
        sleep(250);

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
        telemetry.addData("Ready for Autonomous", "............");

//        while(!gamepad1.y) {

        telemetry.addData("Ready for Autonomous", "............");
        telemetry.addData("voltage", "%.1f volts", new Func<Double>() {
            @Override
            public Double value() {
                return getBatteryVoltage();
            }
        });
        telemetry.update();
//        }

        //double imuData = 0;
        //format IMU Gyro angle unit to double
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        // double value headingDouble will use for Bot front direction
        headingDouble = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                (angles.angleUnit, angles.firstAngle));

        //shooter control thread
//        ShootingCtrl shootingCtrl = new ShootingCtrl();//driveTrainRdy, wobbleClawRdy, shooterRdy,

        //wobble control thread
//        WobbleClawCtrl wobbleClawCtrl = new WobbleClawCtrl();//(driveTrainRdy, wobbleClawRdy, shooterRdy,

        //Start image recognition
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5,//1,//1.5,//2.5,//3.5,
                    16.0 / 9.0);
        }

        double pid_p = 10;//1.26;
        double pid_i = 3;//0.126;
        double pid_d = 1;//0.3;//2;//0.5;//0;
        double pid_f = 12.6;
        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        //Use #getPIDFCoefficients(RunMode) instead

//        PIDFCoefficients pidOrig = motorShooterR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorShooterR.setVelocityPIDFCoefficients(pid_p,pid_i,pid_d,pid_f);
//        motorShooterL.setVelocityPIDFCoefficients(pid_p,pid_i,pid_d,pid_f);
//        motorShooterR.setPositionPIDFCoefficients(5.0);
//        motorShooterL.setPositionPIDFCoefficients(5.0);
        // re-read coefficients and verify change.
//        PIDFCoefficients pidModified = motorShooterR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

//        telemetry.addData("P,I,D,F (orig) :", "%.04f, %.04f, %.04f, %.04f",
//                pidOrig.p, pidOrig.i, pidOrig.d,pidOrig.f);
//        telemetry.addData("P,I,D,F (modified) :", "%.04f, %.04f, %.04f, %.04f",
//                pidModified.p, pidModified.i, pidModified.d, pidModified.f);
//        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            double tfodStartTime = getRuntime();
            while (opModeIsActive() && ((getRuntime() - tfodStartTime) < 0.5)) {// 0.5)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0) {
                            // empty list.  no objects recognized.
                            configSetup = 1;
                            telemetry.addData("TFOD", "No items detected.");
                            telemetry.addData("Target Zone", "A");
                            telemetry.addData("configSetup", "%5d", configSetup);

                        } else {
                            // list is not empty.
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
//                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                        recognition.getLeft(), recognition.getTop());
//                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                        recognition.getRight(), recognition.getBottom());

                                telemetry.addData("  Top - Bottom : ", "%.03f ",
                                        (recognition.getBottom() - recognition.getTop()));

                                telemetry.addData("  Left - Right : ", "%.03f ",
                                        (recognition.getLeft() - recognition.getRight()));

                                telemetry.addData("  Aspect Ratio : ", "%.03f ",
                                        recogRingAspectRatio);

//                                recogRingHeight = (recogRingHeight + (recognition.getBottom() - recognition.getTop()))/2;
                                recogRingHeight = (recognition.getBottom() - recognition.getTop());
                                recogRingAspectRatio = (recognition.getBottom() - recognition.getTop()) / (recognition.getRight() - recognition.getLeft());
                                //new added
                                // check label to see which target zone to go after.
//                                if (recognition.getLabel().equals("Single")) {
                                if (//recognition.getLabel().equals("Quad") &&
                                        (recogRingAspectRatio) < 0.6) {//90) {
                                    configSetup = 2;
                                    telemetry.addData("Target Zone", "B");
                                    telemetry.addData("configSetup", "%5d", configSetup);
                                } else if (//recognition.getLabel().equals("Quad") &&
                                        (recogRingAspectRatio >= 0.7)) {//90)) {
                                    configSetup = 3;
                                    telemetry.addData("Target Zone", "C");
                                    telemetry.addData("configSetup", "%5d", configSetup);
                                } else {
                                    //If problem happened, go Zone A, at lest deliver two wobbles to
                                    //Zone A for end game scores.
                                    configSetup = 1;
                                    telemetry.addData("Target Zone", "UNKNOWN");
                                    telemetry.addData("configSetup", "%5d", configSetup);
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }

            //Close tfod
            if (tfod != null) {
                tfod.shutdown();
            }
        }

        //DEBUG: pause for object detection

//        while (!gamepad1.y){
//            telemetry.addData("Target Detection Done.", "....");
//            telemetry.addData("configSetup:","%5d", configSetup);
//            telemetry.addData(" "," ");
//            telemetry.addData("Press .y to continue Autonomous..","....");
//            telemetry.update();
//        }

        //Start threads after image recognition is done.
//        wobbleClawCtrl.start();
        shootingCtrl.start();

/***************************************************************************************************
 /
 */


        /************** Zone A ***************************************************************/

        //bot move to target zone base on config setup
        if (configSetup == 1) {
            sleep(0);


            motorWBClaw.setTargetPosition(WOBBLEARM_PICKUP);//800);//750);//(WOBBLEARM_AUTOLIFT);//(900);
            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorWBClaw.setPower(0.8);

            wobbleClawRdy = false;
            // battery Voltage
            batteryVoltage = getBatteryVoltage();

            bumperServo.setPosition(BUMPER_SERVO_DOWN);

            //enable shooter thread to turn on shooter motore before move bot
            wobbleClawRdy = false;
            wobbleArmEnd = false;
            shooterRdy = true;  // go to 2-1
            //start autonomous route
            //move to 1st shooting position
            driveBySection(0.8,//0.85,
                    44,//22,
                    0,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            /**turn to 1st shooting heading */
            //bot turn to tele initial heading (0 degree)
            //probably bad timing?
            startFlywheel(2000);
            shootLevelServo.setPosition(GbLevelAgj + 0.49);
            motorIntake.setPower(0);

            turnCenter(0.5,
                    GbAngleAdj + 18,
                    5,
                    0.5,//0.5,   //accuracy in degree
                    5,
                    1,
                    1,
                    true);

            sleep(100);

            moveShootServo(300, 300);
            moveShootServo(300, 300);
            moveShootServo(300, 100);

            stopFlywheel();

            turnCenter(0.5,
                    GbAngleAdj + 0,
                    5,
                    0.5,//0.5,   //accuracy in degree
                    5,
                    1,
                    1,
                    true);

            double temp = getRuntime();
            driveBySection(0.8,//0.85,
                    6,//22,
                    0,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );
            while(getRuntime() - temp < 1.5) {}

            //drop wobble
            wobbleServo.setPosition(WOBBLECLAW_OPEN);
            sleep(100);

            driveBySection(0.5,//0.85,
                    -21, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            motorWBClaw.setTargetPosition(WOBBLEARM_INIT);//800);//750);//(WOBBLEARM_AUTOLIFT);//(900);
            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorWBClaw.setPower(0.8);
            wobbleServo.setPosition(WOBBLECLAW_CLOSE);

            driveBySection(0.5,//0.85,
                    20, 50,
                    25, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            motorIntake.setPower(0);
        }

        /**                                End Zone A
         /**====================================================================================*/


        /**======================================================================================
         /*                                Zone B
         /**====================================================================================*/
        else if (configSetup == 2) {

            wobbleClawRdy = false;
            // battery Voltage
            batteryVoltage = getBatteryVoltage();

            bumperServo.setPosition(BUMPER_SERVO_DOWN);

            //enable shooter thread to turn on shooter motore before move bot
            wobbleClawRdy = false;
            wobbleArmEnd = false;
            shooterRdy = true;  // go to 2-1
            //start autonomous route
            //move to 1st shooting position
            driveBySection(0.6,//0.85,
                    34,//22,
                    51,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            /**turn to 1st shooting heading */
            //bot turn to tele initial heading (0 degree)
            //probably bad timing?
            startFlywheel(2000);
            shootLevelServo.setPosition(GbLevelAgj + 0.50);
            motorIntake.setPower(0);

            turnCenter(0.5,
                    GbAngleAdj + 5.5,
                    5,
                    0.5,//0.5,   //accuracy in degree
                    5,
                    1,
                    1,
                    true);

            moveShootServo(300, 300);
            moveShootServo(300, 300);
            moveShootServo(300, 100);

            motorIntake.setPower(1.0);

            driveBySection(0.6,//0.85,
                    6,//22,
                    5.5,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );


            shootLevelServo.setPosition(GbLevelAgj + 0.48);

            double temp = getRuntime();
            turnCenter(0.5,
                    GbAngleAdj + 5.5,
                    5,
                    0.5,//0.5,   //accuracy in degree
                    5,
                    1,
                    1,
                    true);
            while (getRuntime() - temp < 1.5) {}

            moveShootServo(200, 300);
            stopFlywheel();

            //move wobble arm down
            motorWBClaw.setTargetPosition(WOBBLEARM_PICKUP);//800);//750);//(WOBBLEARM_AUTOLIFT);//(900);
            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorWBClaw.setPower(0.8);

            driveBySection(0.5,//0.85,
                    27,//22,
                    -45,//12,//2,
                    24, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            driveBySection(0.4,//0.85,
                    14,//22,
                    55,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );


            //drop wobble
            wobbleServo.setPosition(WOBBLECLAW_OPEN);
            sleep(100);

            driveBySection(0.4,//0.85,
                    -18,//22,
                    0,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            motorWBClaw.setTargetPosition(WOBBLEARM_INIT);//800);//750);//(WOBBLEARM_AUTOLIFT);//(900);
            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorWBClaw.setPower(0.8);
            wobbleServo.setPosition(WOBBLECLAW_CLOSE);

            motorIntake.setPower(0);
        }
        /**======================================================================================
         /*                                End Zone B
         /**======================================================================================*/


        /**======================================================================================
         /*                                 Zone C
         /**======================================================================================*/

        //target Zone C
        else if (configSetup == 3) {

            wobbleClawRdy = false;
            // battery Voltage
            batteryVoltage = getBatteryVoltage();

            bumperServo.setPosition(BUMPER_SERVO_DOWN);

            //enable shooter thread to turn on shooter motore before move bot
            wobbleClawRdy = false;
            wobbleArmEnd = false;
            shooterRdy = true;  // go to 2-1
            //start autonomous route
            //move to 1st shooting position
            driveBySection(0.6,//0.85,
                    34,//22,
                    51,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            /**turn to 1st shooting heading */
            //bot turn to tele initial heading (0 degree)
            //probably bad timing?
            startFlywheel(2000);
            shootLevelServo.setPosition(GbLevelAgj + 0.50);
            motorIntake.setPower(0);

            turnCenter(0.5,
                    GbAngleAdj + 6.5,
                    5,
                    0.5,//0.5,   //accuracy in degree
                    5,
                    1,
                    1,
                    true);

            moveShootServo(400, 400);
            moveShootServo(400, 400);
            moveShootServo(400, 100);

            motorIntake.setPower(1.0);

            driveBySection(1.0,//0.85,
                    6,//22,
                    5.5,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    true,
                    2,
                    3//1
            );

            shootLevelServo.setPosition(GbLevelAgj + 0.49);

            double temp = getRuntime();
            turnCenter(0.5,
                    GbAngleAdj + 6.5,
                    5,
                    0.5,//0.5,   //accuracy in degree
                    5,
                    1,
                    1,
                    true);
            while (getRuntime() - temp < 1.8) {}

            moveShootServo(200, 100);

            sleep(500);

            driveBySection(0.3,//0.85,
                    13,//22,
                    4.5,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );
            driveBySection(0.3,//0.85,
                    -3,//22,
                    4.5,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );
            driveBySection(0.3,//0.85,
                    10,//22,
                    4.5,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            shootLevelServo.setPosition(GbLevelAgj + 0.46);

            temp = getRuntime();
            turnCenter(0.5,
                    GbAngleAdj + 3.5,
                    5,
                    0.5,//0.5,   //accuracy in degree
                    5,
                    1,
                    1,
                    true);
            while (getRuntime() - temp < 1.0) {}


            moveShootServo(300, 300);
            moveShootServo(300, 300);
            moveShootServo(300, 100);

            stopFlywheel();

            //move wobble arm down
            motorWBClaw.setTargetPosition(WOBBLEARM_PICKUP);//800);//750);//(WOBBLEARM_AUTOLIFT);//(900);
            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorWBClaw.setPower(0.8);

            driveBySection(0.6,//0.85,
                    23,//22,
                    -60,//12,//2,
                    36, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            //drop wobble
            wobbleServo.setPosition(WOBBLECLAW_OPEN);
            sleep(100);

            driveBySection(0.6,//0.85,
                    -28,//22,
                    0,//12,//2,
                    0, 0,
                    0, 0,
                    0, 0,
                    0, 0,
                    5,
                    150,//100,//200,
                    1.5,//1,//2,//1.5,
                    1,//1.5,//2,
                    false,
                    false,
                    2,
                    3//1
            );

            motorWBClaw.setTargetPosition(WOBBLEARM_INIT);//800);//750);//(WOBBLEARM_AUTOLIFT);//(900);
            motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorWBClaw.setPower(0.8);
            wobbleServo.setPosition(WOBBLECLAW_CLOSE);

            motorIntake.setPower(0);
        }

        turnCenter(0.5,
                GbAngleAdj + 0,
                5,
                0.5,//0.5,   //accuracy in degree
                5,
                1,
                1,
                true);

        /**************************************************************************************
         End of Zone ABC
         **************************************************************************************/

        if (!opModeIsActive()){
            exitAuto();
        }
        driveTrainRdy = false;

        /**  From W08 */
//        while (!driveTrainRdy) {
//            wobbleClawRdy = true;
//            if (!opModeIsActive()){
//                exitAuto();
//            }
//        }
        /**  From W08 */
        //wait for wobble move end before stop wobble control thread.
        while (!wobbleArmEnd){
            if (!opModeIsActive()){
                exitAuto();
            }
        }

        //stop threads
//        wobbleClawCtrl.Stop();
        shootingCtrl.Stop();
        //interrupt the threads.
//        wobbleClawCtrl.interrupt();
        shootingCtrl.interrupt();
        //end autonomous
    }


    /**
     * driveBySection() use IMU for Robot heading and two Odometries which connected to MRF and MLF
     * to control drive train. It can handle up to 5 section moves with different heading.
     * Speed should be always positive number, distance and angle can be positive or negative number.
     * Odometry counts are reversed to move direction.
     */
    // Return xy offset are int which are motor ticks.
    public void driveBySection ( double speed,      // positive value
                                 double distance1, //can be positive or negative
                                 double angle1,    //positive or negative left + / - right
                                 double distance2,
                                 double angle2,
                                 double distance3,
                                 double angle3,
                                 double distance4,
                                 double angle4,
                                 double distance5,
                                 double angle5,
                                 double driveTime,
                                 int     accuracy,       //accuracy based on odometry counts
                                 double startRatio,
                                 double stopRatio,
                                 boolean skipWeight,
                                 boolean targetAdj,
                                 int    brkOn,
                                 int    brkOff)
    {

        int moveCounts1;
        int moveCounts2;
        int moveCounts3;
        int moveCounts4;
        int moveCounts5;
        int moveCountsTotal;
        double timeDiff = 0;
        double prevTime;
        double currentTime;
        double startTime;
        double max;
        double errorAngle=0;
        double steer=0;
        double errorAngleSum = 0;
        double speedMRF = 0;;
        double speedMRB = 0;;
        double speedMLF = 0;;
        double speedMLB = 0;
        double powerWeight;
        double startPower;
        double py_Coeff = 0;
        double dy_Coeff = 0;
        double pa_Coeff = 0;
        double da_Coeff = 0;
        int opCycleCnt = 0;


        int moveStartPointOdmR= 0;;
        int moveStartPointOdmL= 0;;
        int moveStartPointAvg= 0;;
        int currentPositionOdmR= 0;;
        int currentPositionOdmL= 0;;
        int currentPositionAvg= 0;;
        int prevPositionAvg = 0;
        int newTarget;
        int onTargetYCnt = 0;
        int fineTurnYCnt = 0;
        int moveDistance;
        boolean onTargetY = false;
        double errorY;
        double errorYSum = 0;
        double steerY;
        double Angle = 0;
        double Heading = 0;

        int distanceDiff = 0;
        double speedDriveTrain = 0;
        double powerSpeed;
        int brakeCnt = 0;
        int brakeFactor;

        // Ensure that  opmode is still active
        if (opModeIsActive()) {
            //Odometry counts will not reset after initialized.
            //Current HW configuration both odometries R/L get negative counts when bot move forward

            //Reset Odometry counts
            motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    //odmR move forward get negative counts
            //motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);     //odmL move forward get negative counts
            //motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //set drive train motor stop behavior to float.
//            motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            //set drive train motor stop behavior to float.
            motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set motors mode. Can't not use encoder for drive train motors.
            // Otherwise will results in unexpected operations.
            motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Get start position directly from odometry
            moveStartPointOdmR = -motorRightFront.getCurrentPosition();  //move forward get negative counts
            moveStartPointOdmL = -motorLeftFront.getCurrentPosition();   //move forward get negative counts
            moveStartPointAvg = (moveStartPointOdmR + moveStartPointOdmL) / 2;

            // Determine new target position, and pass to motor controller
            // unify to ticks per turn
            // distances can use negative but speed will use positive.
            moveCounts1 = (int) (distance1 * ODM_COUNTS_PER_INCH);
            moveCounts2 = (int) (distance2 * ODM_COUNTS_PER_INCH) + moveCounts1;
            moveCounts3 = (int) (distance3 * ODM_COUNTS_PER_INCH) + moveCounts2;
            moveCounts4 = (int) (distance4 * ODM_COUNTS_PER_INCH) + moveCounts3;
            moveCounts5 = (int) (distance5 * ODM_COUNTS_PER_INCH) + moveCounts4;
            moveCountsTotal = moveCounts5;

            // Set target positions. Use average counts of both Odometries.
            newTarget = moveStartPointAvg + moveCountsTotal;

            //Set start time.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            startTime = getRuntime();
            prevTime = startTime;
            currentTime = startTime;



            // keep looping while we are still active, and until time out (driveTime).
            while (opModeIsActive() && ((currentTime - startTime) < driveTime) && (!onTargetY))
            {
//                batteryVoltage = voltSensor.getVoltage();

                opCycleCnt++;
                currentPositionOdmR = -motorRightFront.getCurrentPosition(); //Odometry right side
                currentPositionOdmL = -motorLeftFront.getCurrentPosition();  //Odometry left side
                //Use two odometries counts average
                currentPositionAvg = (currentPositionOdmR + currentPositionOdmL) / 2;

                moveDistance = Math.abs(currentPositionAvg - moveStartPointAvg);

                if (moveDistance <= Math.abs(moveCounts1)) {
                    Angle = angle1;
                } else if ((moveDistance > Math.abs(moveCounts1))
                        && (moveDistance <= Math.abs(moveCounts2))) {
                    Angle = angle2;
                } else if ((moveDistance > Math.abs(moveCounts2))
                        && (moveDistance <= Math.abs(moveCounts3))) {
                    Angle = angle3;
                } else if ((moveDistance >= Math.abs(moveCounts3))
                        && (moveDistance <= Math.abs(moveCounts4))) {
                    Angle = angle4;
                } else if ((moveDistance > Math.abs(moveCounts4))
                        && (moveDistance <= Math.abs(moveCounts5))) {
                    Angle = angle5;
                }

                currentTime = getRuntime();
                //Time difference between two cycles.
                timeDiff = (currentTime - prevTime);   //seconds
                prevTime = currentTime;

//


                //Brake ABS function
                brakeFactor = 1;
                distanceDiff = currentPositionAvg - prevPositionAvg;
                prevPositionAvg = currentPositionAvg;
                speedDriveTrain = distanceDiff / timeDiff;  //in ticks
                //Convert to motor power speed
                powerSpeed = speedDriveTrain/(1200*5*4/2.25);

                //before reach the target
                if ((((currentPositionAvg - newTarget) > (-STOP_RANGE) ) && (moveCountsTotal >= 0))
                        || (((currentPositionAvg - newTarget) < STOP_RANGE) && moveCountsTotal < 0))
                {
                    //Brake ABS function
                    // brkOn/brkOff time braking
                    if (Math.abs(powerSpeed) >  ( STOP_POWER -0.1)){
                        brakeCnt++;
                        if (brakeCnt <= brkOn){//2) {
                            //to set 0 power to motor for brake
                            brakeFactor = 0;
                        }
                        else{
                            //to use normal PID control power
                            brakeFactor = 1;
                        }
                        if (brakeCnt == (brkOn + brkOff)) { brakeCnt = 0;}
                    }
                    else{
                        brakeCnt = 0;
                        brakeFactor = 1;
                    }
                }
                else{
                    brakeCnt = 0;
                    brakeFactor = 1;
                }

                //fineTurnYCnt = 0;
                // Close to target, reduce power and enable Y PID control){
                if ((((currentPositionAvg - newTarget) > (-800) ) && (moveCountsTotal >= 0))
                        || (((currentPositionAvg - newTarget) < 800) && moveCountsTotal < 0))
                {

                    // Check Y on target. Y PID control without Angle PID control
                    // Target Y coordinate fine adjust.
                    errorY = newTarget - currentPositionAvg;// + X_ERROR_CORRECT;
                    errorYSum += errorY * timeDiff;
//                    powerWeight = 0.5;//0.4;//0.3;


                    double py_Coeff_ND = 0.004;//0.0035;//0.0025;//0.001;
//                    double py_Coeff = 0;
                    double iy_Coeff = py_Coeff_ND/15;//10;//0;
//                    double dy_Coeff = 0;
                    if (Math.abs(currentPositionAvg - newTarget) < accuracy)//< 50)
//                        &&Math.abs(currentPositionOdmR - newTargetOdmR_Y) < 50))
                    {
                        onTargetYCnt++;
                        // little large powerWeight for final tune
//                        powerWeight = 0.3;

                        if (batteryVoltage > 14){
                            py_Coeff = py_Coeff_ND*0.9*0.9*0.9;
                            iy_Coeff = iy_Coeff*0.9*0.9*0.9;//0.000075*0.9*0.0*0.9;
                            dy_Coeff = 0.00003*0.9*0.9*0.9;
                        }
                        else if ( batteryVoltage > 13.5 && batteryVoltage <=14){
                            py_Coeff = py_Coeff_ND*0.9*0.8;//0.9;
                            iy_Coeff = iy_Coeff*0.9*0.8;//0.9;//0.000075*0.9*0.9;
                            dy_Coeff = 0.00003*0.9*0.9;
                        }
                        else if (batteryVoltage > 13 && batteryVoltage <= 13.5){
                            py_Coeff = py_Coeff_ND*0.8;//0.85;//0.9;
                            iy_Coeff = iy_Coeff*0.8;//0.85;//0.9;//
                            dy_Coeff = 0.00003*0.9;
                        }
                        else if (batteryVoltage <= 13){
                            py_Coeff = py_Coeff_ND*0.85*0.8;//0.9;//0.01;
                            iy_Coeff = iy_Coeff*0.85*0.8;//0.9;//0.000075;
                            dy_Coeff = 0.00003;
                        }


                        //steerY = getPIDSteer(P_Y_COEFF, I_Y_COEFF, 0.00003,
                        steerY = getPIDSteer(
                                py_Coeff,//0.0001,//0.001,//0.005,//0.01,//0.005,//0.01,
                                iy_Coeff,//0,//iy_Coeff,//0,
                                0,//dy_Coeff,//0,//0.0001,//0,
                                errorY, errorYSum, timeDiff);

                        //if (moveCountsTotal < 0) steerY *= -1;
//                      if (Math.abs(steerY) >1) steerY /= Math.abs(steerY);
//                        speedMRF *= steerY;
//                        speedMRB *= steerY;
//                        speedMLF *= steerY;
//                        speedMLB *= steerY;
                        speedMRF = speed * steerY;
                        speedMRB = speed * steerY;
                        speedMLF = speed * steerY;
                        speedMLB = speed * steerY;

                        if (onTargetYCnt >= 6){//10){//5) {
                            //Continue stay on target < 50.
                            //Finished on target, stop motors and exit the loop.
                            motorRightFront.setPower(0);
                            motorRightBack.setPower(0);
                            motorLeftFront.setPower(0);
                            motorLeftBack.setPower(0);
                            onTargetY = true;
//                            while(!gamepad1.y){
//                                telemetry.addData("onTargetYCnt: ", onTargetYCnt);
//                                telemetry.addData("onTargetYCnt break  ", " Exit ...");
//                                telemetry.addData("Push gampad1.y", " to continue");
//                                telemetry.update();
//                            }

//                            break;  // on target, exit loop.
                        }
                    } else {
                        // Target Y coordinate final adjust.
//                    onTargetY = false;
                        fineTurnYCnt++;
                        onTargetYCnt = 0;


                        double py_Coeff_ND1 = 0.004;//0.0035;//0.0025;//0.001;
                        iy_Coeff = py_Coeff_ND1/35;//30;//40;

                        if (batteryVoltage > 14){
                            py_Coeff = py_Coeff_ND1*0.9*0.9*0.9;
                            iy_Coeff = iy_Coeff*0.9*0.9*0.9;//
                            dy_Coeff = 0.00006*0.9*0.9*0.9;

                        }
                        else if ( batteryVoltage > 13.5 && batteryVoltage <=14){
                            py_Coeff = py_Coeff_ND1*0.9*0.9;
                            iy_Coeff = iy_Coeff*0.9*0.9;//
                            dy_Coeff = 0.00006*0.9*0.9;
                        }
                        else if (batteryVoltage > 13 && batteryVoltage <= 13.5){
                            py_Coeff = py_Coeff_ND1*0.8;//0.85;//0.9;
                            iy_Coeff = iy_Coeff*0.8;//0.85;//0.9;//
                            dy_Coeff = 0.00006*0.9;
                        }
                        else if (batteryVoltage <= 13){
                            py_Coeff = py_Coeff_ND1*0.85*0.9;//0.1;
                            iy_Coeff = iy_Coeff*0.85*0.9;////
                            dy_Coeff = 0.00006;
                        }
                        //steerY = getPIDSteer(P_Y_COEFF, I_Y_COEFF, 0.00003,
                        steerY = getPIDSteer(
                                py_Coeff,//0.001,//0.01,//0.1,
                                iy_Coeff,//0,//I_Y_COEFF,
                                0,//dy_Coeff,//0.0,
                                errorY, errorYSum, timeDiff);

                        //if (moveCountsTotal < 0) steerY *= -1;
//                    if (Math.abs(steerY) >1) steerY /= Math.abs(steerY);
                        //speed no angle adjust
//                    speedMRF *= steerY;
//                    speedMRB *= steerY;
//                    speedMLF *= steerY;
//                    speedMLB *= steerY;
                        speedMRF = speed * steerY;
                        speedMRB = speed * steerY;
                        speedMLF = speed * steerY;
                        speedMLB = speed * steerY;
                    }
                }
                else {
                    // Adjust relative speed based on heading error. No y PID.
                    // Get error based on IUM.

                    double pa_Coeff_ND2;
                    //Calculate P based on total move distance
                    if (moveCountsTotal < 24){
                        pa_Coeff_ND2 = 0.05;//0.1;//0.2;//0.1; //start distance
                    }else{
                        pa_Coeff_ND2 = 0.05;//0.1;
                    }


                    if (batteryVoltage > 14){
                        pa_Coeff = pa_Coeff_ND2*0.9*0.9*0.9;
                        da_Coeff = pa_Coeff/60*0.9*0.9*0.9;//

                    }
                    else if ( batteryVoltage > 13.5 && batteryVoltage <=14){
                        pa_Coeff = pa_Coeff_ND2*0.9*0.9;
                        da_Coeff = pa_Coeff/60*0.9*0.9;//0.0003;//0.002;
                    }
                    else if (batteryVoltage > 13 && batteryVoltage <= 13.5){
                        pa_Coeff = pa_Coeff_ND2*0.9;
                        da_Coeff = pa_Coeff/60*0.9;//0.0006;//0.002;
                    }
                    else if (batteryVoltage <= 13){
                        pa_Coeff = pa_Coeff_ND2;//0.06;
                        da_Coeff = pa_Coeff/60;//0.001;//0.002;
                    }
                    errorAngle = getAngleError(Angle);  //from IMU
                    errorAngleSum += errorAngle * timeDiff;
                    steer = getPIDSteer(
                            pa_Coeff,//0.06,//pa_Coeff,//0.06,//0.08
                            0.0,
                            0,//da_Coeff,//0.002,//0.0,
                            errorAngle, errorAngleSum, timeDiff);

                    //steer already clip in getPIDSteer() .
//                if (Math.abs(steer) > 1) steer /= Math.abs(steer);

                    //Keep minimum speed >=0.
                    if (moveCountsTotal >= 0) {
                        speedMRF = speed + steer;
                        speedMRB = speed + steer;
                        speedMLF = speed - steer;
                        speedMLB = speed - steer;
                        if ((steer >= 0) && speedMLF < 0) {
                            speedMLF = 0;
                            speedMLB = 0;
                        } else if ((steer < 0) && speedMRF < 0) {
                            speedMRF = 0;
                            speedMRB = 0;
                        }
                    } else {
                        speedMRF = -speed + steer;
                        speedMRB = -speed + steer;
                        speedMLF = -speed - steer;
                        speedMLB = -speed - steer;
                        if ((steer >= 0) && speedMRF > 0) {
                            speedMRF = 0;
                            speedMRB = 0;
                        } else if ((steer < 0) && speedMLF > 0) {
                            speedMLF = 0;
                            speedMLB = 0;
                        }
                    }
                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(speedMRF), Math.abs(speedMLF));
                    if (max > 1.0) {
                        speedMRF /= max;
                        speedMRB /= max;
                        speedMLF /= max;
                        speedMLB /= max;
                    }

                    // Move distance too small to use weight
//                    boolean skipWeight = false;

                }

                if (!skipWeight) {
                    if (Math.abs(moveCountsTotal) < (SET_POINT + DECREASE_POINT)) {
                        powerWeight = SHORT_DIS_POWER;
                    } else {
                        powerWeight = drivePowerWeight(
                                currentPositionAvg,
                                moveStartPointAvg,
                                newTarget,
                                START_POWER * startRatio,
                                SET_POWER,
                                STOP_POWER * stopRatio,
                                SET_POINT,
                                DECREASE_POINT,
                                STOP_RANGE);
                    }
                }
                else{
                    powerWeight = 1;
                }

                // If no target adjust, once move distance greater than moveCountsTotal, exit loop.
                if ((!targetAdj)
                        && (Math.abs(currentPositionAvg - moveStartPointAvg)
                        >= Math.abs(moveCountsTotal))) {
//                  newTarget = moveStartPointAvg + moveCountsTotal;
                    motorRightFront.setPower(0);
                    motorRightBack.setPower(0);
                    motorLeftFront.setPower(0);
                    motorLeftBack.setPower(0);

                    break;  //exit loop without target position adjust.
                }

                //Adjust drive train power
//                motorRightFront.setPower(speedMRF * powerWeight);
//                motorRightBack.setPower(speedMRB * powerWeight);
//                motorLeftFront.setPower(speedMLF * powerWeight);
//                motorLeftBack.setPower(speedMLB * powerWeight);

                motorRightFront.setPower(speedMRF * powerWeight * brakeFactor);
                motorRightBack.setPower(speedMRB * powerWeight * brakeFactor);
                motorLeftFront.setPower(speedMLF * powerWeight * brakeFactor);
                motorLeftBack.setPower(speedMLB * powerWeight * brakeFactor);

            }
            // Turn to start angle
//            turnCenter(TURN_SPEED, angle1, 5);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);

        }
    }


    public void turnCenter ( double speed,
                             double angle,
                             double driveTime,
                             double accuracy,   //accuracy in degree
                             int    targetAdjCycle,  // within accuracy cycles
                             int    brkOn,
                             int    brkOff,
                             boolean targetAdj)
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
        double pa_Coeff = 0;
        double ia_Coeff = 0;
        double da_Coeff = 0;
        int cycleCnt = 0;
        int whileCnt = 0;
        int fineTurnCnt = 0;

        int brakeCnt = 0;
        int brakeFactor = 1;

        double timeDiffAvg = 0;
        double timeDiffAvgPrev = 0;

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

        //set drive train motor stop behavior to float.
//        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        headingDouble = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                (angles.angleUnit, angles.firstAngle));

        double getPIDsteerP;

        //calculate I
        double currentI = 0;
        double getPIDsteerI;
        if ( (Math.abs(angle - headingDouble) >= 10 ) ){
            getPIDsteerI = 0;
        }
        else {
            getPIDsteerI = 1;
        }

//        ElapsedTime holdTimer = new ElapsedTime();
        startTime = getRuntime();   //seconds
        prevTime = startTime;
        currentTime = startTime;

        //calculate P
//            double getPIDsteerP;
        if ( (Math.abs(angle - headingDouble) <= 3) ) {
            getPIDsteerP = 0.15;//0.2;//0.18;//0.16;//0.20;//0.15;
        }else if ( (Math.abs(angle - headingDouble) < 5) &&(Math.abs(angle - headingDouble) > 3) ) {
            getPIDsteerP = 0.1;//0.11;//0.15;//0.1;
        }else{
            getPIDsteerP = 0.075;//0.1;//0.12;//0.075;
        }

        boolean dirIndicator = true;
//        robotError = targetAngle - headingDouble;
//        while (robotError > 180) robotError -= 360;
//        while (robotError <= -180) robotError += 360;
        //right/left indicator
        if ((angle - headingDouble) > 0 && (angle - headingDouble) < 180){
            //left turn
            dirIndicator = false;
        }
        else if ((angle - headingDouble) > 180){
            //right turn
            dirIndicator = true;
        }
        else if ((angle - headingDouble) < 0 && (angle - headingDouble) > -180){
            //right turn
            dirIndicator = true;
        }
        else if ((angle - headingDouble) <= -180){
            //left turn
            dirIndicator = false;
        }

        double prevError = Math.abs(getAngleError(angle));
        error = getAngleError(angle);

        while (opModeIsActive()
                && (!onTarget)
                && ((currentTime - startTime) < driveTime))
        {
            whileCnt++;

            // Determine turn power based on Bot heading +/- error. Right shift +error, left shift -error
            error = getAngleError(angle); //robotError = targetAngle - headingDouble;

            //Time difference
            currentTime = getRuntime();
            timeDiff = (currentTime - prevTime);
            timeDiffAvg = (timeDiffAvgPrev + timeDiff)/2;   //for display

            prevTime = currentTime;
            timeDiffAvgPrev = timeDiffAvg;


            //When robot move away from target, enable ABS
            //ABS function
            if (Math.abs(error) > prevError){
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


            //If bot heading reach the target angle, exit.
            //Use direction: dirIndicator to estimate reach angle
            if (!targetAdj){
                //no accuracy adjust
                if (Math.abs(error) <= accuracy){
                    //stop and exit loop
                    leftSpeed = 0.0;
                    rightSpeed = 0.0;
                    onTarget = true;
                }
            }
            else {
                //accuracy adjust

                // Bot heading is continuously in threshold 5 cycles. Turn will be done.
//            if (Math.abs(error) <= HEADING_THRESHOLD) {
                if (Math.abs(error) <= 3) {
                    //only small move need to calculate I
                    errorSum += error * timeDiff;
                    if (Math.abs(error) <= accuracy)    //0.25)
                    {
                        fineTurnCnt = 0;

                        ia_Coeff = getPIDsteerP /1;/// 2;///3;
//                    getPIDsteerP = 0.05;//0.01;
                        if (batteryVoltage > 14) {
                            pa_Coeff = getPIDsteerP * 0.9 * 0.9 * 0.9;
                            ia_Coeff = ia_Coeff * 0.9 * 0.9 * 0.9;
                            da_Coeff = 0.0001 * 0.9 * 0.9 * 0.9;
                        } else if (batteryVoltage > 13.5 && batteryVoltage <= 14) {
                            pa_Coeff = getPIDsteerP * 0.9 * 0.9;
                            ia_Coeff = ia_Coeff * 0.9 * 0.9;
                            da_Coeff = 0.0001 * 0.9 * 0.9;
                        } else if (batteryVoltage > 13 && batteryVoltage <= 13.5) {
                            pa_Coeff = getPIDsteerP * 0.9;
                            ia_Coeff = ia_Coeff * 1;//0.9;
                            da_Coeff = 0.0001 * 0.9;
                        } else if (batteryVoltage > 12.5 && batteryVoltage <= 13) {
                            pa_Coeff = getPIDsteerP * 0.95;
                            ia_Coeff = ia_Coeff * 0.95;//0.1;
                            da_Coeff = 0.0001;
                        } else if (batteryVoltage > 12 && batteryVoltage <= 12.5) {
                            pa_Coeff = getPIDsteerP / 1;
                            ia_Coeff = ia_Coeff / 1;
                            da_Coeff = 0.0001*0.9;
                        } else if (batteryVoltage < 12) {
                            pa_Coeff = getPIDsteerP / 0.9 ;
                            ia_Coeff = ia_Coeff / 0.9 / 0.9;//0.1;
                            da_Coeff = 0.0001/ 0.9;
                        }

                        steerTurn = getPIDSteer(
                                pa_Coeff,//getPIDsteerP,//0.15,
                                ia_Coeff,//0,//ia_Coeff,
                                0,//da_Coeff,//0,//da_Coeff,
                                error, errorSum, timeDiff);

                        //steerTurn cliped in getPIDSteer()
                        //rightSpeed = Range.clip(speed * steerTurn, -1, 1);
                        rightSpeed = speed * steerTurn;// /2;
                        leftSpeed = -rightSpeed;
//                    steerTurn = 0.0;
//                    leftSpeed = 0.0;
//                    rightSpeed = 0.0;

//                    if (cycleCnt > 6){//5) {
                        if (cycleCnt >= targetAdjCycle) {//5) {
                            steerTurn = 0.0;
                            leftSpeed = 0.0;
                            rightSpeed = 0.0;
                            onTarget = true;
                            //break;
                        }
                        cycleCnt++;
                    } else {
                        //speed = speed/2;
                        fineTurnCnt++;
                        cycleCnt = 0;
                        //currentI = getPIDsteerI*0.02;

                        if (fineTurnCnt <= 40) {
                            currentI = 0.25;//0.15;//0.25;//getPIDsteerI*0.25;
                            //currentI = 0.02;
                        } else if ((fineTurnCnt > 40) && (fineTurnCnt <= 100)) {
                            currentI = 0.2;//0.12;//0.2;//0.12;
                        } else if ((fineTurnCnt > 100) && (fineTurnCnt <= 150)) {
                            currentI = 0.15;//0.1;//0.15;
                        } else if (fineTurnCnt > 150) {
                            currentI = 0.12;//0.1;//0.12;//0.2;
                        }


                        pa_Coeff = getPIDsteerP * 1.1;//1.2;
                        ia_Coeff = currentI;//*1.1;///1.5;//3;//10;
//                    getPIDsteerP = 0.05;//0.01;
                        if (batteryVoltage > 14) {
                            pa_Coeff = pa_Coeff * 0.9 * 0.9 * 0.9;
                            ia_Coeff = ia_Coeff * 0.9 * 0.9 * 0.9;//currentI*0.9*0.9*0.9;
                            da_Coeff = 0.00001 * 0.9 * 0.9 * 0.9;//0.002;

                        } else if (batteryVoltage > 13.5 && batteryVoltage <= 14) {
                            pa_Coeff = pa_Coeff * 0.9 * 0.9;
                            ia_Coeff = ia_Coeff * 0.9 * 0.9;//currentI*0.9*0.9;
                            da_Coeff = 0.00001 * 0.9 * 0.9;//0.002;
                        } else if (batteryVoltage > 13 && batteryVoltage <= 13.5) {
                            pa_Coeff = pa_Coeff * 0.9;
                            ia_Coeff = ia_Coeff * 1;//0.9;//currentI*0.9;
                            da_Coeff = 0.00001 * 0.9;//0.002;
                        }
//                    else if (batteryVoltage <= 13){
//                        pa_Coeff = pa_Coeff;
//                        ia_Coeff = ia_Coeff;//currentI;
//                        da_Coeff = 0.00001;//0.002;
//                    }
                        else if (batteryVoltage > 12.5 && batteryVoltage <= 13) {
                            pa_Coeff = pa_Coeff * 0.95;
                            ia_Coeff = ia_Coeff * 1;
                            da_Coeff = 0.0001;
                        } else if (batteryVoltage > 12 && batteryVoltage <= 12.5) {
                            pa_Coeff = pa_Coeff;//
                            ia_Coeff = ia_Coeff;//
                            da_Coeff = 0.0001 /1;
                        } else if (batteryVoltage < 12) {
                            pa_Coeff = pa_Coeff;//
                            ia_Coeff = ia_Coeff;//
                            da_Coeff = 0.0001/ 0.9;
                        }

                        steerTurn = getPIDSteer(
                                pa_Coeff,//getPIDsteerP,//0.1,//0.05,//getPIDsteerP,
                                ia_Coeff,//0,//ia_Coeff,//currentI,//0.05,
                                0,//da_Coeff,//0.00001,//0,//0.00005,
                                error, errorSum, timeDiff);


                        //steerTurn clipped in getPIDSteer()
                        //rightSpeed = Range.clip(speed * steerTurn, -1, 1);
                        rightSpeed = speed * steerTurn;
                        leftSpeed = -rightSpeed;
//                    cycleCnt = 0;
                        onTarget = false;
                    }
                } else {
                    //Don't count large error move I
                    errorSum = 0;

//                getPIDsteerP = 0.05;//0.01;
                    if (batteryVoltage > 14) {
                        pa_Coeff = getPIDsteerP * 0.9 * 0.9 * 0.9;//0.02;
                        da_Coeff = 0.00001;//0.002;

                    } else if (batteryVoltage > 13.5 && batteryVoltage <= 14) {
                        pa_Coeff = getPIDsteerP * 0.9 * 0.8;
                        da_Coeff = 0.00002;//0.002;
                    } else if (batteryVoltage > 13 && batteryVoltage <= 13.5) {
                        pa_Coeff = getPIDsteerP * 0.9;//0.8;
                        da_Coeff = 0.000035;//0.002;
                    }
//                else if (batteryVoltage <= 13){
//                    pa_Coeff = getPIDsteerP;//0.1;
//                    da_Coeff = 0.00005;//0.002;
//                }
                    else if (batteryVoltage > 12.5 && batteryVoltage <= 13) {
                        pa_Coeff = getPIDsteerP * 1;
//                    ia_Coeff = ia_Coeff;//0.1;
                        da_Coeff = 0.00005;
                    } else if (batteryVoltage > 12 && batteryVoltage <= 12.5) {
                        pa_Coeff = getPIDsteerP;
                        ia_Coeff = ia_Coeff;
                        da_Coeff =  0.00005;
                    } else if (batteryVoltage < 12) {
                        pa_Coeff = getPIDsteerP / 0.9;
                        ia_Coeff = ia_Coeff / 0.9;
                        da_Coeff = 0.00005/ 0.9;
                    }

                    steerTurn = getPIDSteer(
                            pa_Coeff,
                            0.0,
                            da_Coeff,//0,
                            error, errorSum, timeDiff);
                    //steerTurn cliped in getPIDSteer()
                    //rightSpeed = Range.clip(speed * steerTurn, -1, 1);
                    rightSpeed = speed * steerTurn;
                    leftSpeed = -rightSpeed;
                    cycleCnt = 0;
                    fineTurnCnt = 0;
                    onTarget = false;
                }
            }

            // Send desired turn speeds to motors.
            motorRightFront.setPower(rightSpeed * brakeFactor);
            motorRightBack.setPower(rightSpeed * brakeFactor);
            motorLeftFront.setPower(leftSpeed * brakeFactor);
            motorLeftBack.setPower(leftSpeed * brakeFactor);

//            telemetry.addData("turnCenter() ", "....");
//            telemetry.addData("batteryVoltage: ", batteryVoltage);
//            telemetry.addData("Angle error: ", error);
//            telemetry.addData("timeDiff: ", timeDiff);
//            telemetry.addData("timeDiffAvg: ", timeDiffAvg);
//            telemetry.addData("dirIndicator: ", dirIndicator);
//            telemetry.update();

        }
        // Stop motors when exit.
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);

        //set drive train motor stop behavior to float.
//        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        headingDouble = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                (angles.angleUnit, angles.firstAngle));
        robotError = targetAngle - headingDouble;
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

    public double drivePowerWeight ( int currentPosition,
                                     int moveStartPint,
                                     int targetPosition,
                                     double start_POWER,
                                     double set_POWER,
                                     double stop_POWER,
                                     int set_POINT,
                                     int decrease_POINT,
                                     int stop_RANGE)
    {
        /**public double drivePowerWindow() constance
         static final double START_POWER = 0.1;  //wheel start power
         static final int SET_POINT = 1200/2;  //half turn of wheel
         static final double SET_POWER = 1;  //setPower;
         static final int DECREASE_POINT = 12000/4; // 1/4 turn of wheel
         static final double STOP_POWER = 0.5;   //wheel stop power
         */
        double powerWeightLocal;
        int movePosition;

        movePosition = Math.abs(currentPosition - moveStartPint);
        targetPosition = Math.abs(targetPosition - moveStartPint);

        // First condition includes currentPosition beyond moveStartPoint.
        if ((movePosition >= 0) && (movePosition < set_POINT)) {
            // Start up and boost
            powerWeightLocal = (set_POWER - start_POWER) / set_POINT * movePosition + start_POWER;
//            powerWeightLocal = SET_POWER;
        } else if ((movePosition >= set_POINT)
                && ((targetPosition - movePosition) >= decrease_POINT)) {
            // Max power
            powerWeightLocal = set_POWER;
        } else if (((Math.abs(targetPosition) - movePosition) < decrease_POINT)
                && ((Math.abs(targetPosition) - movePosition) >= stop_RANGE))
//                  && (movePosition <= Math.abs(targetPosition)))
//        else
        {
            // Decrease power
            // Power weight start with (SET_POWER-0.4)
            powerWeightLocal = (stop_POWER - (set_POWER)) / decrease_POINT * movePosition +
                    (stop_POWER - (stop_POWER - (set_POWER))
                            / decrease_POINT * targetPosition);
        } else if (((Math.abs(targetPosition) - movePosition) < stop_RANGE)
                && (movePosition <= Math.abs(targetPosition))) {
            // Final range set to stop_power
            powerWeightLocal = stop_POWER;
        } else {
            // In case move over target position
            powerWeightLocal = stop_POWER;
        }

        return powerWeightLocal;
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    /**********************   ShootingCtrl Thread   ***********************************/
    public class ShootingCtrl extends Thread {
        //Thread run condition
        private boolean shooterIsRunning = true;    //shooter thread is running

        boolean shootLaunchRdy = false;
        int motorSpeedSet = MotorSpeedSet+100;//+50;
        int motorSpeedMax = motorSpeedSet+50;
        int motorSpeedMin = motorSpeedSet-50;
        int speedCycleCnt = 0;

        public ShootingCtrl() {
            this.setName("ShootingCtrl Thread.");
        }

        /**
         * Stops the shooter thread
         */
        public void Stop(){ shooterIsRunning = false; }


        /**
         * Runs the thread
         */
        @Override
        public void run() {
            //while (!isInterrupted()) {
            while (!isInterrupted() && shooterIsRunning) {
                try {

                    /** 2-1 */
                    while (!shooterRdy) {
                        telemetry.addData("shooter thread ", "wait for start.");
                        telemetry.addData("shooterRdy", shooterRdy);
                        telemetry.update();
                    }


                    shootServo.setPosition(SHOOTPARK);
                    //delay after bot start moving
                    sleep(1000);//600);//1000);
                    motorShooterR.setVelocity(motorSpeedSet);// +50);
                    motorShooterL.setVelocity(motorSpeedSet);// +50);

                    shooterRdy = false;

                    //shooter motors speeds in right range
                    speedCycleCnt = 0;
                    shootLaunchRdy = false;

                    //set shoot level for 1st power shoot
                    shootLevelServo.setPosition(0.465);//0.485);//0.49);//0.52);
                    /** 1st shooting */
                    while (!shooterRdy) {

                    }
                    /**
                     while ((!shooterRdy) || (!shootLaunchRdy)) {
                     if ((motorShooterL.getVelocity() < motorSpeedMin)
                     || (motorShooterL.getVelocity() > motorSpeedMax)) {
                     speedCycleCnt = 0;
                     shootLaunchRdy = true;//for testing false;
                     } else {
                     //stay on speed range for >500 cycles
                     speedCycleCnt++;
                     if (speedCycleCnt >= 10) {
                     shootLaunchRdy = true;
                     speedCycleCnt = 0;
                     }
                     }
                     }
                     */


                    //1st shooting
                    shootServo.setPosition(SHOOTING);
                    sleep(300);

                    /**enable Bot to turn to 2nd shooting */
                    driveTrainRdy = true;   //to 1-3


                    //move shooter arm to park
                    shootServo.setPosition(SHOOTPARK);
                    sleep(300);

                    shooterRdy = false;

                    /** 2nd shooting
                     */
                    speedCycleCnt = 0;
                    shootLaunchRdy = false;

                    //set shoot level for 2nd power shoot
//                    shootLevelServo.setPosition(0.485);//0.49);//0.485);//0.49);//0.51);

                    /** 2nd shooting */
                    /** S03 */
                    while (!shooterRdy) {

                    }
                    /**
                     while ((!shooterRdy) || (!shootLaunchRdy)) {
                     if ((motorShooterL.getVelocity() < motorSpeedMin)
                     || (motorShooterL.getVelocity() > motorSpeedMax)) {
                     speedCycleCnt = 0;
                     shootLaunchRdy = true;//for testing false;false;
                     } else {
                     //                            stay on speed range for >500 cycles
                     speedCycleCnt++;
                     if (speedCycleCnt >= 25) {
                     shootLaunchRdy = true;
                     speedCycleCnt = 0;
                     }
                     }
                     }
                     */
                    //2nd shooting
                    shootServo.setPosition(SHOOTING);
                    sleep(300);

                    driveTrainRdy = true; //to 1-4

                    shootServo.setPosition(SHOOTPARK);

                    sleep(300);

                    shooterRdy = false;

                    /*****************************************
                     *  Wait until shooter motor speed stable
                     *  3rd shooting
                     */
                    speedCycleCnt = 0;
                    shootLaunchRdy = false;

                    //set shoot level for 3rd power shoot
//                    shootLevelServo.setPosition(0.49);//0.485);//0.49);//0.505);

                    while (!shooterRdy) {

                    }
                    /**
                     while ((!shooterRdy) || (!shootLaunchRdy)) {
                     if ((motorShooterL.getVelocity() < motorSpeedMin)
                     || (motorShooterL.getVelocity() > motorSpeedMax)) {
                     speedCycleCnt = 0;
                     shootLaunchRdy = true;//for testing false;false;
                     } else {
                     //                            stay on speed range for >500 cycles
                     speedCycleCnt++;
                     if (speedCycleCnt >= 20) {
                     shootLaunchRdy = true;
                     speedCycleCnt = 0;
                     }
                     }
                     }
                     */
                    //3rd shooting
                    shootServo.setPosition(SHOOTING);
                    sleep(300);

                    /** Go to M01*/
                    driveTrainRdy = true;   //M01

                    motorShooterL.setPower(0);
                    motorShooterR.setPower(0);
                    shootServo.setPosition(SHOOTPARK);
//                        sleep(300);
                    sleep(300);

                    shooterRdy = false;
//                        sleep(300);

                    /**==============Zone ABC share code end  ===============================*/
/**

 /************** Zone A **************************************************/
                    //Target Zone A
                    if (configSetup == 1) {

                        wobbleClawRdy = true;   //to
                        //turn off shooting motor
                        motorShooterL.setPower(0);
                        motorShooterR.setPower(0);

                        shootServo.setPosition(SHOOTPARK);
//                    try {
                        sleep(300);

                        shooterRdy = false;

                        //end shooting control thread.
                        shooterIsRunning = false;


                        //End of zone A shooting
                    }
                    /************** Zone B ***************************************************/
                    //Target Zone B
                    else if (configSetup == 2) {

                        /**  S021 */
                        shooterRdy = false;

                        /**  From M2-1 */
                        while (!shooterRdy) {
                        }

                        //Shooter motors turn in main thread.
                        //Shooting
                        shootServo.setPosition(SHOOTING);
                        sleep(250);

                        //turn on motors
//                        motorShooterL.setPower(0);
//                        motorShooterR.setPower(0);
                        motorShooterR.setVelocity(0);
                        motorShooterL.setVelocity(0);

                        /**  S022 */
                        //enable bot move
                        driveTrainRdy = true;

                        //turn off intake
                        motorIntake.setPower(0);

                        /**  S023 */
                        //enable claw control
                        wobbleClawRdy = true;

                        //park position
                        shootServo.setPosition(SHOOTPARK);
                        sleep(400);

                        shooterRdy = false;

                        //end shooting control thread.
                        shooterIsRunning = false;

                    }
                    /************** Zone C ***************************************************/
                    //Target Zone C
                    //from M01
                    else {  //configSetup == 3

                        motorShooterL.setPower(0);
                        motorShooterR.setPower(0);
//                        shootServo.setPosition(SHOOTPARK);
//                        sleep(300);

                        shooterRdy = false;

                        /**  S031 */
                        while (!shooterRdy) {
                        }
                        //wait for 3 rings go to position
//                        sleep(200);
                        //turn off intake
                        sleep(250);//300);//250);//200);
                        motorIntake.setPower(0);
                        sleep(200);//100);//200);
                        //continue 3 shoots
                        shootServo.setPosition(SHOOTING);
                        sleep(200);//230);
                        //park position
                        shootServo.setPosition(SHOOTPARK);
                        sleep(300);//230);//180);//200);//150);//400);
                        shootServo.setPosition(SHOOTING);
                        sleep(200);//230);
                        //park position
                        shootServo.setPosition(SHOOTPARK);
//                        sleep(180);//200);//150);//400);
//                        sleep(400);
//                        shootServo.setPosition(SHOOTING);
//                        sleep(200);//230);
//                        park position
//                        shootServo.setPosition(SHOOTPARK);

                        /**  S032 */
                        driveTrainRdy = true;
                        //turn on intake
                        motorIntake.setPower(1);

                        motorShooterL.setPower(0);
                        motorShooterR.setPower(0);
//                        //park position
//                        shootServo.setPosition(SHOOTPARK);

                        //delay to make sure 4th ring are on the position
//                        sleep(40000);//3500);//3000);//4000);//2000);//4000);

                        shooterRdy = false;

                        /**  S033 */
                        while (!shooterRdy) {
                        }
                        //delay to make sure 4th ring are on the position
//                        sleep(500);//2000);//3500);//3000);//4000);//2000);//4000);

                        //make sure shooter is turned on
//                        motorShooterL.setPower(1);
//                        motorShooterR.setPower(1);
                        //Make last shooting
                        //delay to turn off intake
//                        sleep(500);//3000);
//                        motorIntake.setPower(0);
                        //delay for shooter stable

                        sleep(350);//300);

                        motorIntake.setPower(0);

                        sleep(150);//200);//100);//200);

                        shootServo.setPosition(SHOOTING);
                        sleep(200);//250);
                        //park position
                        shootServo.setPosition(SHOOTPARK);
                        sleep(350);//300);
                        //take one more shooting in case first 3rd shoot miss one ring
                        shootServo.setPosition(SHOOTING);
                        sleep(200);//250);


//                        motorIntake.setPower(0);

                        /**  S034 */
                        driveTrainRdy = true;

                        //Go to W01 of wobble claw control thread
                        /**  S035 Go to W01 */
                        wobbleClawRdy = true;
                        //park position
                        shootServo.setPosition(SHOOTPARK);

                        sleep(100);
                        //turn off shooter
                        motorShooterL.setPower(0);
                        motorShooterR.setPower(0);

                        shooterRdy = false;

                        //end of Zone C shooting

                    }


                    //end shooting control thread.
                    shooterIsRunning = false;

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }

    /**********************   WobbleClawCtrl Thread   ***********************************/
//    public class WobbleClawCtrl extends Thread{
//
//        //Thread run condition
//        private boolean wbcIsRunning = true;
//
//        public WobbleClawCtrl() {
//            this.setName("WobbleClawCtrl Thread.");
//        }
//        /**
//         * Stops the shooter thread
//         */
//        public void Stop() {
//            wbcIsRunning = false;
//        }
//
//        /**
//         * Runs the thread
//         */
//        @Override
//        public void run() {
////            while (wbcIsRunning)
//            while (!isInterrupted() && wbcIsRunning)
////            while (wbcIsRunning)
//            {
//                try {
//                    wobbleClawRdy = false;
//                    /**  From S023 */
//                    /**  W01 from S035*/
//                    while (!wobbleClawRdy) {
//                        //wait
//                    }
//
//                    motorWBClaw.setTargetPosition(WOBBLEARM_AUTOLIFT);//800);//750);//(WOBBLEARM_AUTOLIFT);//(900);
//                    motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motorWBClaw.setPower(0.8);
//                    while (motorWBClaw.isBusy()) {
//                        //wait for ARM move to target position.
//                    }
//                    //Need to finish claw arm move before Bot arrive drop location.
//                    wobbleClawRdy = false;
//
//                    /**  From M2-1 */
//                    /**  w02 */
//                    while (!wobbleClawRdy) {}
//
//                    //drop wobble
//                    wobbleServo.setPosition(WOBBLECLAW_OPEN);
////
//                    sleep(200);//100);
////
//                    /**  W03 */
//                    driveTrainRdy = true;
//
//                    sleep(500);
//
//                    //move arm to pick up after drop wobble
//                    motorWBClaw.setTargetPosition(WOBBLEARM_PICKUP);
//                    motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motorWBClaw.setPower(0.7);
//                    while (motorWBClaw.isBusy()) {
//                        //wait for ARM move to target position.
//                    }
//
//                    sleep(300);     //make sure drive train starts
//                    //keep arm at pick up position and claw open for 2nd wobble pick up
//                    wobbleClawRdy = false;
//
//                    /**  W04 */
//                    /**  From M2-2 */
//                    while (!wobbleClawRdy) {
//                        //wait..
//                        //driveTrainRdy = true;
//                    }
//                    //pick up 2nd wobble
//                    //claw close in two step to handle wide pick up
//                    wobbleServo.setPosition(WOBBLECLAW_CLOSE +0.4);//+0.1);//+0.2);
//                    sleep(400);//500);//300);
//
//                    wobbleServo.setPosition(WOBBLECLAW_CLOSE);
//                    sleep(250);//300);//400);
//
//                    /**  W05 */
//                    driveTrainRdy = true;
//
//                    //move arm to low hold position
//                    motorWBClaw.setTargetPosition(WOBBLEARM_AUTOLIFT);//(900);
//                    motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motorWBClaw.setPower(0.9);
//                    while (motorWBClaw.isBusy()) {
//                        //wait for ARM move to target position.
//                    }
//                    wobbleClawRdy = false;
//
//                    /**  W06 */
//                    /**  From M2-3 */
//                    //moving to drop location. Drop wobble at WOBBLEARM_AUTOLIFT position.
//                    while (!wobbleClawRdy) {
//                        //driveTrainRdy = true;
//                    }
//                    //drop 2nd wobble
//                    wobbleServo.setPosition(WOBBLECLAW_OPEN);
//                    sleep(300);//100);//200);
//
//                    /**  W07 */
//                    driveTrainRdy = true;
//
//                    //add more delay in case wobble hold is not in correct position
//                    sleep(200);//500);
//
//                    //Zone B don't need move back to end line center.
////                    if (configSetup == 2) {
////                        move arm to tele position and close claw
////                        motorWBClaw.setTargetPosition(WOBBLEARM_PICKUP);//WOBBLEARM_TELEPARK);
////                        motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                        motorWBClaw.setPower(0.5);//1);
////                        while (motorWBClaw.isBusy()) { }
////                    }
////                    else{
//
//                    //move arm to init position and close claw
//                    motorWBClaw.setTargetPosition(WOBBLEARM_INIT);//WOBBLEARM_TELEPARK);
//                    motorWBClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    motorWBClaw.setPower(0.9);//0.5);//1);
//                    while (motorWBClaw.isBusy()) {
//                        //set claw to park position while arm moving
//                        sleep(300);
//                        wobbleServo.setPosition(WOBBLECLAW_CLOSE);
//                        //telemetry.addData("Claw moving to Tele position.", "Wait.. ");
//                        //telemetry.update();
//                    }
////                    }
//
////                    wobbleClawRdy = false;
//
//                    /**  W08 */
//                    //make sure wobble arm go to position before end thread in main thread.
////                    driveTrainRdy = true;
//
//                    //end WobbleClawCtrl Thread
////                    wbcIsRunning = false;
//
//                    wobbleClawRdy = false;
//                    //end WobbleClawCtrl Thread
//                    wbcIsRunning = false;
//
//                    /**  W08 */
//                    //notify main thread wobble move control end.
//                    wobbleArmEnd = true;
//                    //make sure wobble arm go to position before end thread in main thread.
////                    driveTrainRdy = true;
//
//                } catch (Exception e) {
//                    e.printStackTrace();
//                }
//            }
//        }
//    }
//

    /**
     *
     * Exit Autonomous
     */
    // Return xy offset are int which are motor ticks.
    public void exitAuto ( )
    {
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        //stop threads
//        wobbleClawCtrl.Stop();
        shootingCtrl.Stop();
        //interrupt the threads.
//        wobbleClawCtrl.interrupt();
        shootingCtrl.interrupt();
    }
}
