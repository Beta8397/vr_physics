package org.firstinspires.ftc.teamcode.util.gamepad;

/**
 * Abstract class that implements toggle functionality to a boolean input, typically a gamepad
 * button.
 */
public abstract class ButtonToggle {

    /**
     * Types of toggle event that can be handled by ButtonToggle
     */
    public enum Mode {RELEASED, CHANGED, PRESSED}

    private Mode mode;
    private boolean priorState = false;

    public ButtonToggle(Mode toggleMode){
        mode = toggleMode;
    }

    /**
     * Subclass must provide a getButtonState() method that returns the current state of the button
     * to be monitored by the ButtonToggle object.
     * @return Current state of the button being monitored by the ButtonToggle object.
     */
    protected abstract boolean getButtonState();

    /**
     * Updates the ButtonToggle object based on current state of the monitored button, AND
     * returns a value indicating whether a toggle event has occurred.
     * @return true if a toggle event has occurred, otherwise false.
     */
    public boolean update(){
        boolean currentState = getButtonState();
        boolean result = false;
        switch (mode) {
            case RELEASED:
                result = priorState? !currentState : false;
                break;
            case CHANGED:
                result = currentState != priorState;
                break;
            case PRESSED:
                result = priorState? false : currentState;
                break;
        }
        priorState = currentState;
        return result;
    }

}
