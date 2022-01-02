package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurningPID {
    // NOTE:
    // the old PID code modified the drivetrain motor modes and zero power behaviors before running the loop
    // I have left that part out since I think they set it to the value already in use
    // and because i dont know where I would put that code.

    public double speed;
    public double targetAngleErrorMargin;

    public int abs_BrakeDuration;
    public int abs_NoBrakeDuration;

    public int onTargetCyclesNeeded = 4; // constant across all applications and normally 4
    // do we need to make this modifiable?

    TurningPID(double newSpeed, double newTargetErrorMarginAngle, int newABS_BrakeDuration, int newABS_NoBrakeDuration) {
        setParams(newSpeed, newTargetErrorMarginAngle);
        setABSParams(newABS_BrakeDuration, newABS_NoBrakeDuration);
    }

    public void setParams(double newSpeed, double newTargetErrorMarginAngle) {
        speed = newSpeed;
        targetAngleErrorMargin = newTargetErrorMarginAngle;
    }

    public void setABSParams(int newABS_BrakeDuration, int newABS_NoBrakeDuration) {
        abs_BrakeDuration = newABS_BrakeDuration;
        abs_NoBrakeDuration = newABS_NoBrakeDuration;
    }

    public void resetPIDProcess() {
        prevError = 0;
        prevAbsError = 0;
        errorSum = 0;

        cycleCount = 0;
        fineTurnCycleCount = 0;
        brakeCycleCount = 0;

        onTargetCycleCount = 0;
    }

    public boolean isOnTarget() {
        return onTargetCycleCount > onTargetCyclesNeeded;
    }

    public boolean isAlmostOnTarget() {
        return isOnTarget() || (fineTurnCycleCount > onTargetCyclesNeeded);
    }

    double prevError = 0;
    double prevAbsError = 0;
    double errorSum = 0;

    int cycleCount = 0;
    int fineTurnCycleCount = 0;
    int brakeCycleCount = 0;

    int onTargetCycleCount = 0;

    double composePID( double P, double I, double D, double error, double prevError, double errorSum, double deltaTime ) {
        return Range.clip(P * error + I * errorSum + D * (error-prevError)/deltaTime, -1, 1);
    }

    // deltaTime as parameter or deltaTime managed by this class?
    public double getTurnSpeed( double currentAngle, double targetAngle, double deltaTime ) {
        cycleCount ++;

        double var_P;
        double var_I;
        double var_D;

        double error = AngleUnit.DEGREES.normalize(targetAngle - currentAngle);
        double absError = Math.abs(error);
        errorSum += error * deltaTime;

        if (absError <= 3) {
            var_P = 0.15;
        } else if (absError < 5) {
            var_P = 0.12;
        } else {
            var_P = 0.1;
        }


        if (absError > 3) {
            // if angle is way off, ignore I and D calculations
            errorSum = 0;
            onTargetCycleCount = 0;
            fineTurnCycleCount = 0;

            var_I = 0;
            var_D = 0.0005;
        } else {
            var_D = 0.0001;

            if (absError < targetAngleErrorMargin) {
                // on target - onTargetCycleCount will keep counting up
                onTargetCycleCount ++;
                fineTurnCycleCount = 0;

                var_I = 0.2;
            } else {
                // when not on target but within 3 deg - var_I varies w/ time
                onTargetCycleCount = 0;
                fineTurnCycleCount ++;

                if (fineTurnCycleCount < 40){
                    var_I = 0.25;
                } else if (fineTurnCycleCount < 100) {
                    var_I = 0.2;
                } else if (fineTurnCycleCount < 150) {
                    var_I = 0.15;
                } else {
                    var_I = 0.12;
                }
            }
        }

        prevError = error;
        prevAbsError = absError;

//         if error has increased, we use ABS (anti-lock braking system) (for some reason) as we slow down.
//         if braking, the final speed is 0.
        if (absError > prevAbsError) {
            brakeCycleCount ++;
            if (cycleCount % (abs_NoBrakeDuration + abs_BrakeDuration) <= abs_BrakeDuration) {return 0.0;}
        } else {
            brakeCycleCount = 0;
        }

        return speed * composePID(var_P, var_I, var_D, error, 0, errorSum, deltaTime);
    }
}
