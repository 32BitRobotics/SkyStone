package org.firstinspires.ftc.teamcode;

public final class Toggle {
    private boolean value;
    private boolean lastInput;

    public Toggle(boolean value) {
        this.value = value;
        this.lastInput = false;
    }

    public Toggle() {
        this(true);
    }

    public boolean getValue() { return value; }

    public void setInput(boolean input) {
        if (lastInput) {
            if (input) {
                return;
            } else {
                lastInput = false;
            }
        } else {
            if (input) {
                value = true;
                lastInput = true;
            } else {
                return;
            }
        }
    }
}
