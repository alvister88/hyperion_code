package org.firstinspires.ftc.teamcode;

class BasicButtonWatcher {
    int pressCount = 0;

    public boolean isPressed() { return pressCount > 0; }
    public boolean isJustPressed() { return pressCount == 1; }

    public void update(boolean pressed) {
        pressCount = pressed ? pressCount + 1 : 0;
    }
}
