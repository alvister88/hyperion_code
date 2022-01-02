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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="TeleTest", group="TeleOp ...")

public class TeleTest extends LinearOpMode{
    DcMotorEx motorRightFront;  //odometry right side odmR
    DcMotorEx motorLeftFront;  //odometry left side odmL
    DcMotorEx motorRightBack;   //no encoder
    DcMotorEx motorLeftBack;    //no encoder

    DcMotorEx motorFlywheelRight;
    DcMotorEx motorFlywheelLeft;
    Servo servoShootLevel;
    Servo servoIndexer;

    DcMotorEx motorIntake;

    Servo wobbleServo;

    Servo bumperServo;
    double bumperValue = 0;

    final double turnCoeff = 1.0;

    // IMU variables
    BNO055IMU imu;
    // Orientation imuOrientation; //keeping this so I remember how to fall back to old updateIMUVariables().
    double imuHeading = 0;

    double thrustX;
    double thrustY;
    double thrustR;
    double shiftScale;

    boolean leftStickActive;
    boolean rightStickActive;
    boolean leftTriggerPressed;
    boolean rightTriggerPressed;

    boolean leftTriggerPush = false;


    double DEBUG_TEMP = 0;
    int DEBUG_TEMP_2 = 0;
    int DEBUG_TEMP_3 = 0;

    public enum ShootingPosition{MIDDLE, OPPOSITE, BACK, SAFETY}
    ShootingPosition currentShootingPosition = ShootingPosition.MIDDLE;

    public enum IntakeState{FORWARD, STOPPED, REVERSE}
    IntakeState intakeState = IntakeState.STOPPED;
    double reverseIntakeTimestamp = 0;

    Params_Tele_Base teamParams;

    boolean flywheelOn = false;
    boolean isShooting = false;

    //bad
    double currentShootingAngle = 0.0;
    double currentShootingLevel = 0.5;
    int currentShootingSpeedOffset = 0;

    double highGoalGlobalAngleOffset = 0;
    double highGoalGlobalLevelOffset = 0;

    double timer_prevTime = 0;
    double timer_currentTime = 0;
    double timer_elapsedTime = 0;

    double high_goal_shot_timestamp = -1000.0;
    double high_goal_aim_timestamp = -1000.0;
    int high_goal_rings_shot = 0;

    boolean clawControlsEnabled = false;

    ButtonWatcher x_watcher = new ButtonWatcher();
    BasicButtonWatcher dpad_up_watcher = new BasicButtonWatcher();
    BasicButtonWatcher dpad_down_watcher = new BasicButtonWatcher();
    BasicButtonWatcher dpad_left_watcher = new BasicButtonWatcher();
    BasicButtonWatcher dpad_right_watcher = new BasicButtonWatcher();
    BasicButtonWatcher back_watcher = new BasicButtonWatcher();


    public enum TeamColor{RED, BLUE};
    TeamColor currentTeamColor;

    OdoLocalization odometryThread = new OdoLocalization();
    TurningPID pidManager = new TurningPID(0.55, 0.75, 5, 1);

    void composeDTControls() {
        thrustX = Range.clip(-1, thrustX, 1);
        thrustY = Range.clip(-1, thrustY, 1);
        thrustR = Range.clip(-1, thrustR, 1);

        double dtPowerLF = thrustY+thrustX + turnCoeff * thrustR;
        double dtPowerLB = thrustY-thrustX + turnCoeff * thrustR;
        double dtPowerRF = thrustY-thrustX - turnCoeff * thrustR;
        double dtPowerRB = thrustY+thrustX - turnCoeff * thrustR;

        if(Math.abs(dtPowerLF) > 1.0
                || Math.abs(dtPowerLB) > 1.0
                || Math.abs(dtPowerRF) > 1.0
                || Math.abs(dtPowerRB) > 1.0) {

            double maxVal = Math.max(Math.abs(dtPowerLF), Math.abs(dtPowerLB));
            maxVal = Math.max(maxVal, Math.abs(dtPowerRF));
            maxVal = Math.max(maxVal, Math.abs(dtPowerRB));

            dtPowerLF/=maxVal;
            dtPowerLB/=maxVal;
            dtPowerRF/=maxVal;
            dtPowerRB/=maxVal;
        }

        motorLeftFront.setPower(dtPowerLF);
        motorLeftBack.setPower(dtPowerLB);
        motorRightFront.setPower(dtPowerRF);
        motorRightBack.setPower(dtPowerRB);
    }

    void initialize() {
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "MLF");// Right Odometry
        motorLeftBack = hardwareMap.get(DcMotorEx.class,"MLB"); // Back Odometry
        motorRightFront = hardwareMap.get(DcMotorEx.class,"MRF");// Left Odometry
        motorRightBack = hardwareMap.get(DcMotorEx.class,"MRB");

        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);  //Odometry right side. Move forward odometry get negative counts
        motorRightBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorEx.Direction.FORWARD);   //odometry left side. Move forward odometry get negative counts
        motorLeftBack.setDirection(DcMotorEx.Direction.FORWARD);

        //reset encoder
        motorRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  //odomety right side
        motorRightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);   //odometry left side

        motorRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motorRightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorFlywheelRight = hardwareMap.get(DcMotorEx.class, "ShootR");
        motorFlywheelLeft = hardwareMap.get(DcMotorEx.class, "ShootL");
        motorFlywheelRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFlywheelLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorFlywheelRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheelLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheelRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFlywheelLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        wobbleServo = hardwareMap.servo.get("WBGriper");

        servoShootLevel = hardwareMap.servo.get("ShootLevel");
        servoIndexer = hardwareMap.servo.get("shootServo");

        motorIntake = hardwareMap.get(DcMotorEx.class, "Intake");
        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);

        bumperServo = hardwareMap.servo.get("blockArmR");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        currentTeamColor = getTeamColor();

        telemetry.addLine(currentTeamColor == TeamColor.BLUE ? "Blue" : "Red");
        telemetry.update();

        if (currentTeamColor == TeamColor.BLUE) {
            teamParams = new Params_Tele_Blue();
        } else {
            teamParams = new Params_Tele_Red();
        }

        // NASTY //
        odometryThread.motorLeftFront = motorLeftFront;
        odometryThread.motorRightFront = motorRightFront;
        odometryThread.motorRightBack = motorRightBack;
        odometryThread.start();
    }
    void close() {
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);

        odometryThread.stopThreadLoop();
        odometryThread.interrupt();
    }
    void updateTelemetry() {
        telemetry.addData("Press Time 1: ", "%7.3f", x_watcher.pressTimes[0]);
        telemetry.addData("Press Time 2: ", "%7.3f", x_watcher.pressTimes[1]);
        telemetry.addData("Press Time 3: ", "%7.3f", x_watcher.pressTimes[2]);
        telemetry.addLine(x_watcher.isHeld() ? "Held" : "Not Held");
        telemetry.addData("Double Taps: ", "%7d", DEBUG_TEMP_2);
        telemetry.addData("Triple Taps: ", "%7d", DEBUG_TEMP_3);

        telemetry.addData("Main Loop Cycle Time: ", "%7.3f", timer_elapsedTime);
        telemetry.addData("Heading: ", "%7.3f", imuHeading);
        telemetry.addData("PID Output:", "%7.3f", DEBUG_TEMP);
        telemetry.addData("ErrorSum:", "%7.3f", pidManager.errorSum);

        telemetry.addData("Encoder Ticks Left: ", "%7d", odometryThread.getEncoderValueRight());
        telemetry.addData("Encoder Ticks Right: ", "%7d", odometryThread.getEncoderValueLeft());
        telemetry.addData("Encoder Ticks Back: ", "%7d", odometryThread.getEncoderValueBack());
        telemetry.addData(" ", " ");

        telemetry.addData("Global Heading: ", "%7.3f", odometryThread.getLocalizedHeading());
        telemetry.addData("Global X: ", "%7.3f", odometryThread.getLocalizedX());
        telemetry.addData("Global Y: ", "%7.3f", odometryThread.getLocalizedY());

        telemetry.addData(" ", " ");
        telemetry.addData("Cycle Time Avg: ", "%7.4f", odometryThread.getDebug_avgElapsedTime());
        telemetry.addData("Cycle Time Max: ", "%7.4f", odometryThread.getDebug_maxElapsedTime());
        telemetry.addData("Cycle Time Min: ", "%7.4f", odometryThread.getDebug_minElapsedTime());

        telemetry.update();
    }

    /**** necessary? why not just update on every loop cycle ****/
    void updateIMUVariables() {
//        imuOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
//                (imuOrientation.angleUnit, imuOrientation.firstAngle));
        imuHeading = AngleUnit.DEGREES.normalize(
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
                        .firstAngle
        );
    }

    void startFlywheel(int speedOffset) {
        flywheelOn = true;
        motorFlywheelRight.setVelocity(Params_Common.BASE_SHOOTER_RPM + speedOffset);
        motorFlywheelLeft.setVelocity(Params_Common.BASE_SHOOTER_RPM + speedOffset);
    }
    void stopFlywheel() {
        flywheelOn = false;
        motorFlywheelRight.setVelocity(0);
        motorFlywheelLeft.setVelocity(0);
    }

    // I have if statements for all of these since I don't like calling setPower on every loop cycle
    // I don't know if it matters though
    void startIntake() {
        if (intakeState != IntakeState.FORWARD) {
            intakeState = IntakeState.FORWARD;
            motorIntake.setPower(1.0);
        }
    }
    void stopIntake() {
        if (intakeState != IntakeState.STOPPED) {
            intakeState = IntakeState.STOPPED;
            motorIntake.setPower(0);
        }
    }
    void reverseIntake() {
        if (intakeState != IntakeState.REVERSE) {
            intakeState = IntakeState.REVERSE;
            motorIntake.setPower(-1.0);
        }
        thrustY -= 0.2;
    }

    void processServo() {
        if (timer_currentTime - high_goal_shot_timestamp < 0.15) {
            servoIndexer.setPosition(Params_Common.INDEXER_EXTENDED_VALUE);
        } else {
            servoIndexer.setPosition(Params_Common.INDEXER_RETRACTED_VALUE);
        }
    }

    void addJoystickControls() {
        shiftScale = rightTriggerPressed ? 1 - 0.7 * gamepad1.right_trigger : 1;

        thrustX += gamepad1.left_stick_x * shiftScale;
        thrustY += -gamepad1.left_stick_y * shiftScale;

        thrustR += isShooting || leftTriggerPressed ? 0 : gamepad1.right_stick_x * shiftScale;
    }

    TeamColor getTeamColor() {
        while( !isStopRequested() ) {
            if (gamepad1.x) return TeamColor.BLUE;
            if (gamepad1.b) return TeamColor.RED;
        }
        return TeamColor.BLUE;
    }

    void setShootingParameters() {
        switch (currentShootingPosition) {
            case MIDDLE: {
                currentShootingAngle = teamParams.HIGH_GOAL_ANGLE_A;
                currentShootingLevel = teamParams.HIGH_GOAL_LEVEL_A;
                currentShootingSpeedOffset = teamParams.HIGH_GOAL_SPEED_OFFSET_A;
                break;
            }
            case BACK: {
                currentShootingAngle = teamParams.HIGH_GOAL_ANGLE_B;
                currentShootingLevel = teamParams.HIGH_GOAL_LEVEL_B;
                currentShootingSpeedOffset = teamParams.HIGH_GOAL_SPEED_OFFSET_B;
                break;
            }
            case SAFETY: {
                currentShootingAngle = teamParams.HIGH_GOAL_ANGLE_C;
                currentShootingLevel = teamParams.HIGH_GOAL_LEVEL_C;
                currentShootingSpeedOffset = teamParams.HIGH_GOAL_SPEED_OFFSET_C;
                break;
            }
            case OPPOSITE: {
                currentShootingAngle = teamParams.HIGH_GOAL_ANGLE_D;
                currentShootingLevel = teamParams.HIGH_GOAL_LEVEL_D;
                currentShootingSpeedOffset = teamParams.HIGH_GOAL_SPEED_OFFSET_D;
                break;
            }
            default: break;
        }
    }

    void updateControls() {
        x_watcher.update(timer_currentTime, gamepad1.x);
        dpad_up_watcher.update(gamepad1.dpad_up);
        dpad_down_watcher.update(gamepad1.dpad_down);
        dpad_left_watcher.update(gamepad1.dpad_left);
        dpad_right_watcher.update(gamepad1.dpad_right);
        back_watcher.update(gamepad1.back);
    }

    @Override
    public void runOpMode () {
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            timer_currentTime = getRuntime();
            timer_elapsedTime = timer_currentTime - timer_prevTime;

            leftStickActive = Math.abs(gamepad1.left_stick_x) < 0.1 && Math.abs(gamepad1.left_stick_y) < 0.1;
            rightStickActive = Math.abs(gamepad1.right_stick_x) < 0.1;
            leftTriggerPressed = gamepad1.left_trigger > 0.5; // less sensitivity for shooting
            rightTriggerPressed = gamepad1.right_trigger > 0.1;

            updateControls();
            if (x_watcher.isJustSinglePressed()) DEBUG_TEMP_2 ++;
            if (x_watcher.isJustDoublePressed()) DEBUG_TEMP_3 ++;

            thrustX = 0;
            thrustY = 0;
            thrustR = 0;

            if (dpad_up_watcher.isJustPressed()) highGoalGlobalLevelOffset -= 0.01;
            if (dpad_down_watcher.isJustPressed()) highGoalGlobalLevelOffset += 0.01;
            if (dpad_left_watcher.isJustPressed()) highGoalGlobalAngleOffset += 0.5;
            if (dpad_right_watcher.isJustPressed()) highGoalGlobalAngleOffset -= 0.5;

            // Intake Control
            if (!isShooting) {
                if (gamepad1.right_stick_y < -0.5) {
                    // right stick up --> start intake
                    if (intakeState != IntakeState.FORWARD) {
                        startIntake();
                    }
                }
                else if (gamepad1.right_stick_y > 0.5) {
                    // right stick down --> reverses intake, and intake stays stopped when stick is released.
                    // I add a 0.1 second delay before reversing for a tiny bit of smoothness.
                    if (intakeState == IntakeState.FORWARD) {
                        reverseIntakeTimestamp = timer_currentTime;
                    }
                    if (timer_currentTime - reverseIntakeTimestamp > 0.1) {
                        reverseIntake();
                    } else {
                        stopIntake();
                    }
                } else {
                    if (intakeState == IntakeState.REVERSE) {
                        stopIntake();
                    }
                }
            }

            // Shooting Position selection
            // shitty code holy fuck
            if (!gamepad1.left_bumper && !clawControlsEnabled) {
                if (gamepad1.y) {
                    currentShootingPosition = ShootingPosition.MIDDLE;
                    setShootingParameters();
                } else if (gamepad1.x) {
                    currentShootingPosition = ShootingPosition.OPPOSITE;
                    setShootingParameters();
                } else if (gamepad1.b) {
                    currentShootingPosition = ShootingPosition.SAFETY;
                    setShootingParameters();
                } else if (gamepad1.a) {
                    currentShootingPosition = ShootingPosition.BACK;
                    setShootingParameters();
                }
            }

            // Aiming / Shooting controls
            isShooting = false;

            if (leftTriggerPressed) {
                if (!leftTriggerPush) {
                    pidManager.resetPIDProcess();
                    startFlywheel(currentShootingSpeedOffset);
                    leftTriggerPush = true;
                    high_goal_aim_timestamp = timer_currentTime;
                }

                updateIMUVariables();

                servoShootLevel.setPosition(currentShootingLevel + highGoalGlobalLevelOffset + gamepad1.right_stick_y * 0.03);

                DEBUG_TEMP = pidManager.getTurnSpeed(imuHeading, currentShootingAngle + highGoalGlobalAngleOffset, timer_elapsedTime);
                thrustR -= DEBUG_TEMP;

                telemetry.addData("Target Angle: ", "%7.3f", currentShootingAngle + highGoalGlobalAngleOffset);
                telemetry.addData("Target Level: ", "%7.3f", currentShootingLevel + highGoalGlobalLevelOffset + gamepad1.right_stick_y * 0.03);

                // Shooting controls
                if (gamepad1.right_bumper) {
                    if (pidManager.isOnTarget() || (isShooting && pidManager.isAlmostOnTarget())) {
                        isShooting = true;
                        // longer wait time after first ring so the player can see how to adjust the shot
                        double wait_time = high_goal_rings_shot == 1 ? 0.6 : 0.3;

                        if (timer_currentTime - high_goal_shot_timestamp > wait_time && timer_currentTime - high_goal_aim_timestamp > 0.5) {
                            high_goal_shot_timestamp = timer_currentTime;
                            high_goal_rings_shot ++;
                        }
                    }

                }

            } else {
                if (leftTriggerPush) {
                    leftTriggerPush = false;
                    stopFlywheel();
                    pidManager.resetPIDProcess();
                }
                high_goal_rings_shot = 0;
            }

            if (back_watcher.isJustPressed()) { clawControlsEnabled = !clawControlsEnabled; }


            processServo();
            updateTelemetry();

            timer_prevTime = timer_currentTime;

            addJoystickControls();
            composeDTControls();
        }
        close();
    }
}
