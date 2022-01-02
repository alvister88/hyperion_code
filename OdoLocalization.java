package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

class OdoLocalization extends Thread {
    private boolean isRunning = true;

    public DcMotorEx motorLeftFront;
    public DcMotorEx motorRightFront;
    public DcMotorEx motorRightBack;

    /***** Make all variables private? *****/

    private ElapsedTime runTime = new ElapsedTime();
    double debug_elapsedTime;
    double debug_prevTime;

    double debug_avgElapsedTime;
    double debug_avgElapsedTimeSum;
    double debug_minElapsedTime;
    double debug_maxElapsedTime;
    int debug_loopCycle;

    // change in encoder
    double encoderDistanceLeft;
    double encoderDistanceRight;
    double encoderDistanceBack;

    int encoderValueLeft;
    int encoderValueRight;
    int encoderValueBack;
    int prevEncoderValueLeft;
    int prevEncoderValueRight;
    int prevEncoderValueBack;

    double localizedX = 0;
    double localizedY = 0;
    double localizedHeading = 0;

    double temp1, temp2, temp3;

    // change privacy please
    final double radiusY = 6.8231; // (perpendicular) distance from Left/Right wheels to center of rotation
    final double radiusX = 6.25;// (perpendicular) distance from Back wheel to center of rotation

    public final double inchesPerTick = 96.0 / 15850.0;

    void processChange() {
        encoderValueLeft = -motorLeftFront.getCurrentPosition();
        encoderValueRight = -motorRightFront.getCurrentPosition();
        encoderValueBack = -motorRightBack.getCurrentPosition();

        encoderDistanceLeft = (encoderValueLeft - prevEncoderValueLeft) * inchesPerTick;
        encoderDistanceRight = (encoderValueRight - prevEncoderValueRight) * inchesPerTick;
        encoderDistanceBack = (encoderValueBack - prevEncoderValueBack) * inchesPerTick;

        prevEncoderValueLeft = encoderValueLeft;
        prevEncoderValueRight = encoderValueRight;
        prevEncoderValueBack = encoderValueBack;

        double dTheta = 0.5 * (encoderDistanceRight - encoderDistanceLeft) / radiusY;
        double distanceY = 0.5 * (encoderDistanceRight + encoderDistanceLeft);
        // + or - ?
        double distanceX = encoderDistanceBack + (radiusX * dTheta);

        double distance = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
        double strafeAngle = Math.atan2(distanceX, distanceY);

        double curveDisplacementX, curveDisplacementY;
        if (dTheta < 0.002) {
            curveDisplacementX = - 0.5 * distance * dTheta;
            curveDisplacementY = distance;
        } else {
            double curveRadius = distance / dTheta;
            curveDisplacementX = curveRadius * (Math.cos(dTheta) - 1.0);
            curveDisplacementY = curveRadius * Math.sin(dTheta);
        }

        double tempCos = Math.cos(localizedHeading + strafeAngle);
        double tempSin = Math.sin(localizedHeading + strafeAngle);

        temp1 = dTheta;
        temp2 = strafeAngle;

        localizedX += curveDisplacementX * tempCos - curveDisplacementY * tempSin;
        localizedY += curveDisplacementY * tempCos + curveDisplacementX * tempSin;
        localizedHeading += dTheta;
    }

    public synchronized double getLocalizedHeading(){ return localizedHeading; }
    public synchronized double getLocalizedX(){ return localizedX; }
    public synchronized double getLocalizedY(){ return localizedY; }
    public synchronized int getEncoderValueLeft(){ return encoderValueLeft; }
    public synchronized int getEncoderValueRight(){ return encoderValueRight; }
    public synchronized int getEncoderValueBack(){ return encoderValueBack; }

    public synchronized double getEncoderDistanceLeft() { return encoderDistanceLeft; }
    public synchronized double getEncoderDistanceRight() { return encoderDistanceRight; }
    public synchronized double getEncoderDistanceBack() { return encoderDistanceBack; }

    public synchronized int getDebug_loopCycle() { return debug_loopCycle; }
    public synchronized double getDebug_minElapsedTime() { return debug_minElapsedTime; }
    public synchronized double getDebug_maxElapsedTime() { return debug_maxElapsedTime; }
    public synchronized double getDebug_avgElapsedTime() { return debug_avgElapsedTime; }

    public synchronized double getT1() { return temp1; }
    public synchronized double getT2() { return temp2; }
    public synchronized double getT3() { return temp3; }

    public void resetLocalizedCoordinates(){
        localizedX = 0;
        localizedY = 0;
        localizedHeading = 0;

        // necessary?
        prevEncoderValueLeft = 0;
        prevEncoderValueRight = 0;
        prevEncoderValueBack = 0;

        motorRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopThreadLoop() {
        isRunning = false;
    }

    void initDebugVars() {
        runTime.reset();
        debug_loopCycle = 0;
        debug_maxElapsedTime = -69.0;
        debug_minElapsedTime = 69.0;
        debug_avgElapsedTimeSum = 0.0;
        debug_prevTime = 0;
    }

    void processDebugVars() {
        debug_loopCycle ++;
        debug_elapsedTime = runTime.milliseconds() - debug_prevTime;
        if (debug_elapsedTime > debug_maxElapsedTime) { debug_maxElapsedTime = debug_elapsedTime; }
        if (debug_elapsedTime < debug_minElapsedTime) { debug_minElapsedTime = debug_elapsedTime; }
        debug_avgElapsedTimeSum += debug_elapsedTime;

        if (debug_loopCycle % 50 == 0) {
            // every 50 cycles, calculate average and reset variables.
            debug_avgElapsedTime = debug_avgElapsedTimeSum / 50.0;
            debug_minElapsedTime = debug_elapsedTime;
            debug_maxElapsedTime = debug_elapsedTime;
            debug_avgElapsedTimeSum = 0;
        }

        debug_prevTime = runTime.milliseconds();
    }

    @Override
    public void run() {
        initDebugVars();

        while (isRunning) {
            processDebugVars();
            processChange();
        }
    }

}