package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="LinearSlide_VelDrive3", group="TeleOp ...")
public class LinearSlide_VelDrive3 extends LinearOpMode{
    private final ElapsedTime     runtime = new ElapsedTime();
    private final ElapsedTime     carousel_timer = new ElapsedTime();
    private final ElapsedTime     cycle_timer = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    // DT stuff
    static final double DT_SPEED = 2600.0;


    DcMotorEx motorRightFront;
    DcMotorEx motorLeftFront;
    DcMotorEx motorRightBack;
    DcMotorEx motorLeftBack;

    DcMotorEx motorINTK;
    DcMotorEx motorLSF;
    DcMotorEx motorLSD;
    DcMotorEx motorCRSL;

    Servo slideServo;

    boolean LeftTriggerOn = false;
    boolean RightTriggerOn = false;

    boolean button_y_pressed = false;
    boolean button_a_pressed = false;
    boolean button_x_pressed = false;
    boolean button_lb_pressed = false;

    double left_y;
    double left_x;
    double right_y;
    double right_x;

    int newTarget1;
    int newTarget2;

    double lockedServoPosition = 0;
    boolean lockedServo = false;

    int holdPositionLSF = -2000;
    int holdPositionLSD;
    int initialPosition;
    int nextInitialPosition = 0;

    enum slideMove{UP, DOWN}
    slideMove direction = slideMove.UP;

    @Override
    public void runOpMode (){
        boolean runCRSL = false;
        double crslPower = 0.2;
        double crslIncrement = 0.1;

        // controls carousel initial speed and acceleration
        // 0.5 means that power goes from 0 to 0.5 in 1 second.
        final double carousel_base_speed = 0.1;
        final double carousel_acceleration = 0.3;

        boolean runINTK = false;

        double slideServoPosition;

        double mrfVelocity;
        double mrbVelocity;
        double mlfVelocity;
        double mlbVelocity;

        boolean runSlide = false;
        boolean runServos = false;
        double prevTime = 0.0;

        initRobotHW();

        telemetry.addData("Status", "Initialized");

        waitForStart();

        double motorPower = 0;
//        double motorPower = 0;

        while (opModeIsActive()) {
            cycle_timer.reset();
            LeftTriggerOn = Math.abs(gamepad1.left_trigger) > 0.1;
            RightTriggerOn = Math.abs(gamepad1.right_trigger) > 0.1;

            if (gamepad1.x){
                if (!button_x_pressed){
                    button_x_pressed = true;
                    runSlide = !runSlide;

                    if (runSlide) {
                        motorLSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorLSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        direction = slideMove.UP;
                        newTarget1 = 1370;
                        newTarget2 = 0;

                    } else {
                        motorLSF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLSD.setPower(0);
                        motorLSF.setPower(0);
                    }
                }
                else{
                    button_x_pressed = false;
                }
            }

            telemetry.addData("motorLSF position", motorLSF.getCurrentPosition());
            if (runSlide) {
                telemetry.addData("direction", direction);
                switch (direction) {
                    case UP:
                        //if (motorLSF.getCurrentPosition() >= (newTarget1 - 100) && motorLSF.getCurrentPosition() <= (newTarget1 + 100)
                        //&& motorLSD.getCurrentPosition() >= (-newTarget1 - 100) && motorLSD.getCurrentPosition() <= (-newTarget1 + 100) {
                        if ((Math.abs(motorLSF.getCurrentPosition() - newTarget1) <= 50) && (Math.abs(motorLSD.getCurrentPosition() - newTarget1) <= 50)){
                            direction = slideMove.DOWN;
                            prevTime = getRuntime();
                        }
                        else{
                            if((getRuntime() - prevTime) >= 1.0) {
                                motorLSF.setTargetPosition(newTarget1);
                                motorLSD.setTargetPosition(-newTarget1);

                                motorLSF.setPower(0.8);
                                motorLSD.setPower(0.8);
                            }
                        }
                        break;
                    case DOWN:
                        //if (motorLSF.getCurrentPosition() >= (newTarget2 - 20) && motorLSF.getCurrentPosition() <= (newTarget2 + 100)
                        //&& motorLSD.getCurrentPosition() >= (-newTarget2 - 20) && motorLSD.getCurrentPosition() <= (-newTarget2 + 100) {
                        if ((Math.abs(motorLSF.getCurrentPosition() - newTarget2) <= 50) && (Math.abs(motorLSD.getCurrentPosition() - newTarget2) <= 50)) {
                            direction = slideMove.UP;
                            prevTime = getRuntime();
                        }
                        else{
                            if((getRuntime() - prevTime) >= 1.0) {
                                motorLSF.setTargetPosition(newTarget2);
                                motorLSD.setTargetPosition(-newTarget2);

                                motorLSF.setPower(0.8);
                                motorLSD.setPower(0.8);
                            }
                        }
                        break;
                }
            }

            if (-gamepad1.right_stick_y != 0){
                // initialPosition = nextInitialPosition;
                holdPositionLSF = -2000; // theres definitely a better way to do this
                motorLSF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorLSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorLSF.setPower(-gamepad1.right_stick_y/3);
                motorLSD.setPower(gamepad1.right_stick_y/3);
            }
            else if (holdPositionLSF == -2000 && gamepad1.right_stick_y == 0){
                holdPositionLSF = motorLSF.getCurrentPosition();
                holdPositionLSD = motorLSD.getCurrentPosition();
                // nextInitialPosition = holdPositionLSF;
                motorLSF.setTargetPosition(holdPositionLSF);
                motorLSD.setTargetPosition(holdPositionLSD);
                motorLSF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLSF.setPower(0.2);
                motorLSD.setPower(0.2);
            }
            // telemetry.addData("Distance", motorLSF.getCurrentPosition());
            telemetry.update();

            // intake control
            if (gamepad1.y){
                if (!button_y_pressed){
                    runINTK = !runINTK;

                    if (runINTK) {
                        motorINTK.setPower(1.0);

                    } else {
                        motorINTK.setPower(0);
                    }
                }
                button_y_pressed = true;
            } else{
                button_y_pressed = false;
            }

            // carousel control
            if (gamepad1.a) {
                // when button is initially pressed, reset the sequence by resetting the timer.
                if (!button_a_pressed) {
                    carousel_timer.reset();
                }

                // as the button is held, the carousel speeds up
                double carousel_power = carousel_base_speed + carousel_timer.seconds() * carousel_acceleration;
                motorCRSL.setPower(Range.clip(carousel_power, 0, 1.0));

                button_a_pressed = true;
            } else {
                // when button is initially released, the carousel is stopped.
                if (button_a_pressed) {
                    motorCRSL.setPower(0);
                }

                // alternative carousel control using left trigger
                if (LeftTriggerOn) {
                    motorCRSL.setPower(gamepad1.left_trigger);
                }
                else{
                    motorCRSL.setPower(0);
                }

                button_a_pressed = false;
            }

            telemetry.addData("crslPower", motorCRSL.getPower());

            if (gamepad1.left_bumper){
                if (!button_lb_pressed){
                    button_lb_pressed = true;
                    runServos = !runServos;
                }
                else{
                    button_lb_pressed = false;
                }
            }

            // edited this but its probably still messed up
            if (runServos){
                if (gamepad1.right_bumper){
                    lockedServo = true;
                }
                else if (lockedServo && gamepad1.right_trigger == 0){
                    slideServo.setPosition(lockedServoPosition);
                }
                else if (gamepad1.right_trigger > 0){
                    lockedServo = false;
                    slideServo.setPosition(gamepad1.right_trigger);
                    lockedServoPosition = gamepad1.right_trigger;
                }

            }

            // servo control
            if (!runServos) {
                if (gamepad1.dpad_up) {
                    slideServo.setPosition(0.25);
                } else if (gamepad1.dpad_right) {
                    slideServo.setPosition(0.5);
                } else if (gamepad1.dpad_down) {
                    slideServo.setPosition(0.75);
                } else if (gamepad1.dpad_left) {
                    slideServo.setPosition(1);
                } else {
                    slideServo.setPosition(0.0);
                }
            }

            slideServoPosition = slideServo.getPosition();


            // Drivetrain controls
            // TODO: reorganize this stuff
            double turnCoeff = 1.2;
            double shiftScale;

            // left_y flipped so up corresponds w/ positive.
            left_y = -gamepad1.left_stick_y;
            left_x = gamepad1.left_stick_x;

            // right_y flipped so up corresponds w/ positive.
            right_y = -gamepad1.right_stick_y;
            right_x = gamepad1.right_stick_x;

            mlfVelocity = left_y+left_x + turnCoeff * right_x;
            mlbVelocity = left_y-left_x + turnCoeff * right_x;
            mrfVelocity = left_y-left_x - turnCoeff * right_x;
            mrbVelocity = left_y+left_x - turnCoeff * right_x;

            /*
            if(RightTriggerOn) {
                shiftScale = 1.0 -(gamepad1.right_trigger * 0.7);
                mlfVelocity *= shiftScale;
                mlbVelocity *= shiftScale;
                mrfVelocity *= shiftScale;
                mrbVelocity *= shiftScale;
            }
             */


            if(Math.abs(mlfVelocity) > 1.0 || Math.abs(mlbVelocity) > 1.0 || Math.abs(mrfVelocity) > 1.0 || Math.abs(mrbVelocity) > 1.0) {
                double maxVal = Math.max(Math.max(Math.abs(mlfVelocity), Math.abs(mlbVelocity)), Math.max(Math.abs(mrfVelocity), Math.abs(mrbVelocity)));
                mlfVelocity /= maxVal;
                mlbVelocity /= maxVal;
                mrfVelocity /= maxVal;
                mrbVelocity /= maxVal;
            }

            // set the motor speeds
            setDTMotors(mrfVelocity * DT_SPEED, mrbVelocity * DT_SPEED, mlfVelocity * DT_SPEED, mlbVelocity * DT_SPEED);

            telemetry.addData("Status", "Running");
            telemetry.addData("Servo position", slideServoPosition);
            telemetry.addData("Cycle Time (ms)", cycle_timer.milliseconds());
            prevTime = getRuntime();
        }
        stopDTMotors();

    }

    public void initRobotHW(){
        motorRightFront = hardwareMap.get(DcMotorEx.class, "MRF");  // Hub 1 motor port 0
        motorRightBack = hardwareMap.get(DcMotorEx.class, "MRB");  // Hub 1 motor port 1
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "MLF");   // Hub 1 motor port 2
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "MLB");

        motorINTK = hardwareMap.get(DcMotorEx.class, "INTK");
        motorLSF = hardwareMap.get(DcMotorEx.class, "LSF");
        motorLSD = hardwareMap.get(DcMotorEx.class, "LSD");
        motorCRSL = hardwareMap.get(DcMotorEx.class, "CRSL");

        slideServo = hardwareMap.servo.get("grabby");

        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorEx.Direction.FORWARD);

        motorRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorLSF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLSD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLSF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorINTK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCRSL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    void setDTMotors(double RF, double RB, double LF, double LB) {
        motorRightFront.setVelocity(RF);
        motorRightBack.setVelocity(RB);
        motorLeftFront.setVelocity(LF);
        motorLeftBack.setVelocity(LB);
    }

    void stopDTMotors() {
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
    }
}
