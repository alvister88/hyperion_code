package org.firstinspires.ftc.teamcode;

// To Use:

// ButtonWatcher left_bumper_controls = new ButtonWatcher();

// at the beginning of the main loop:
// left_bumper_controls.update(getRuntime(), gamepad1.left_bumper)
class ButtonWatcher {
    int pressCount = 0;
    public double[] pressTimes = {-20.0, -10.0, 0.0};
    double currentTime = 0.0;

    int singlePressCount = 0;
    int doublePressCount = 0;

    double holdDuration = 1.0;
    int holdCount = 0;

    double maxTimeBetweenPresses = 0.4;

    public boolean isPressed() { return pressCount > 0; }
    public boolean isJustPressed() { return pressCount == 1; }

    public boolean isSinglePressed() { return singlePressCount > 0; }
    public boolean isJustSinglePressed() { return singlePressCount == 1; }

    public boolean isDoublePressed() {return doublePressCount > 0; }
    public boolean isJustDoublePressed() {return doublePressCount == 1; }

    public void setHoldDuration(double duration) { holdDuration = duration; }
    public void setMaxTimeBetweenPresses(double duration) { maxTimeBetweenPresses = duration; }

    public boolean isHeld() { return holdCount > 0; }
    public boolean isJustHeld() { return holdCount == 1; }

    public void update(double time, boolean pressed) {
        currentTime = time;
        if (pressed) {
            if (pressCount == 0) addPress();
            if (currentTime - pressTimes[2] > holdDuration) {
                holdCount ++;
            } else {
                holdCount = 0;
            }
            pressCount ++;
            updateDoublePress();
        } else {
            pressCount = 0;
            updateSinglePress();
        }
    }

    void updateSinglePress() {
        if (!isPressed() && currentTime - pressTimes[2] > maxTimeBetweenPresses && pressTimes[2] - pressTimes[1] > maxTimeBetweenPresses) {
            singlePressCount ++;
        } else {
            singlePressCount = 0;
        }
    }

    void updateDoublePress() {
        if (isPressed() && pressTimes[2] - pressTimes[1] < maxTimeBetweenPresses) {
            doublePressCount ++;
        } else {
            doublePressCount = 0;
        }
    }

    void addPress() {
        pressTimes[0] = pressTimes[1];
        pressTimes[1] = pressTimes[2];
        pressTimes[2] = currentTime;
    }
}
