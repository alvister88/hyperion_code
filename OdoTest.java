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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="OdoTest", group="TeleOp ...")

public class OdoTest extends LinearOpMode{
    DcMotorEx motorRightFront;  //odometry right side odmR
    DcMotorEx motorLeftFront;  //odometry left side odmL
    DcMotorEx motorRightBack;   //no encoder
    DcMotorEx motorLeftBack;    //no encoder

    Servo wobbleServo;

    final double turnCoeff = 1.2;

    // IMU variables
    BNO055IMU imu;
    Orientation angleIMU;
    double headingDoubleIMU = 0;

    double thrustX;
    double thrustY;
    double thrustR;

    boolean leftStickActive;
    boolean rightStickActive;
    boolean keyADown = false;

    OdoLocalization odometryThread = new OdoLocalization();

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
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "MLF");//odometry R
        motorLeftBack = hardwareMap.get(DcMotorEx.class,"MLB");
        motorRightFront = hardwareMap.get(DcMotorEx.class,"MRF");//odometry L
        motorRightBack = hardwareMap.get(DcMotorEx.class,"MRB");

        wobbleServo = hardwareMap.servo.get("WBGriper");

        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);  //odomety right side. Move forward odometry get negative counts
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /****** NASTY ******/
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
        telemetry.addData("Encoder Left: ", "%7d", odometryThread.getEncoderValueRight());
        telemetry.addData("Encoder Right: ", "%7d", odometryThread.getEncoderValueLeft());
        telemetry.addData("Encoder Back: ", "%7d", odometryThread.getEncoderValueBack());
        telemetry.addData(" ", " ");

        telemetry.addData("Encoder Ticks Left: ", "%7.3f", odometryThread.getEncoderDistanceLeft() / odometryThread.inchesPerTick);
        telemetry.addData("Encoder Ticks Right: ", "%7.3f", odometryThread.getEncoderDistanceRight() / odometryThread.inchesPerTick);
        telemetry.addData("Encoder Ticks Back: ", "%7.3f", odometryThread.getEncoderDistanceBack() / odometryThread.inchesPerTick);
        telemetry.addData(" ", " ");

        telemetry.addData("Global Heading: ", "%7.3f", odometryThread.getLocalizedHeading());
        telemetry.addData("Global X: ", "%7.3f", odometryThread.getLocalizedX());
        telemetry.addData("Global Y: ", "%7.3f", odometryThread.getLocalizedY());

        telemetry.addData(" ", " ");
        telemetry.addData("Cycle Time Avg: ", "%7.4f", odometryThread.getDebug_avgElapsedTime());
        telemetry.addData("Cycle Time Max: ", "%7.4f", odometryThread.getDebug_maxElapsedTime());
        telemetry.addData("Cycle Time Min: ", "%7.4f", odometryThread.getDebug_minElapsedTime());

        telemetry.addData("dTheta, strafeangle: ", "%7.4f %7.4f", odometryThread.getT1(), odometryThread.getT2());

        telemetry.update();
    }

    /**** necessary? why not just update on every loop cycle ****/
    void updateIMUVariables() {
        angleIMU = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        // double value headingDouble will use for Bot front direction
        headingDoubleIMU = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit
                (angleIMU.angleUnit, angleIMU.firstAngle));
    }
    double getAngleErrorGyro(double targetAngle ) {
        updateIMUVariables();
        return AngleUnit.DEGREES.normalize(targetAngle - headingDoubleIMU);
    }

    @Override
    public void runOpMode () {
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            leftStickActive = Math.abs(gamepad1.left_stick_x) < 0.1 && Math.abs(gamepad1.left_stick_y) < 0.1;
            rightStickActive = Math.abs(gamepad1.right_stick_x) < 0.1;


            double shiftscale = 1;
            if (gamepad1.left_trigger > 0.1) {
                shiftscale = 1 - 0.7 * gamepad1.left_trigger;
            }

            thrustX = gamepad1.left_stick_x * shiftscale;
            thrustY = -gamepad1.left_stick_y * shiftscale;
            thrustR = gamepad1.right_stick_x * shiftscale;

            if(gamepad1.a) {
                if(!keyADown) {
                    // do stuff
                }
                keyADown = true;
            } else {
                keyADown = false;
            }


            if (gamepad1.right_trigger > 0.3) {
                updateIMUVariables();
                double error = getAngleErrorGyro(0);
                double absError = Math.abs(error);
                double turnSpeedCoeff;
                if (absError < 1) {
                    turnSpeedCoeff = 0.05;
                    if (gamepad1.left_bumper) {
                        thrustX += Range.clip(-odometryThread.localizedX/5.0, -1.0, 1.0);
                        thrustY += Range.clip(-odometryThread.localizedY/5.0, -1.0, 1.0);
                    }
                } else if (absError < 10) {
                    turnSpeedCoeff = 0.1;
                    if (gamepad1.left_bumper) {
                        thrustX += Range.clip(-odometryThread.localizedX/5.0, -1.0, 1.0);
                        thrustY += Range.clip(-odometryThread.localizedY/5.0, -1.0, 1.0);
                    }
                } else if (absError < 40) {
                    turnSpeedCoeff = 0.4;
                } else {
                    turnSpeedCoeff = 1;
                }
                thrustR += error < 0 ? turnSpeedCoeff : -turnSpeedCoeff;
            } else if (gamepad1.right_bumper) {
                updateIMUVariables();
                double error = getAngleErrorGyro(0);
                double absError = Math.abs(error);
                double turnSpeedCoeff;
                if (absError < 1) {
                    turnSpeedCoeff = 0.05;
                } else if (absError < 10) {
                    turnSpeedCoeff = 0.1;
                } else if (absError < 40) {
                    turnSpeedCoeff = 0.4;
                } else {
                    turnSpeedCoeff = 1;
                }
            }
            updateTelemetry();
            composeDTControls();
        }
        close();
    }
}
