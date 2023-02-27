package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.hwd_io.util.InvertibleDigitalInput;
import frc.robot.io.joysticks.JS_IO;
import frc.robot.util.Time;

public class Slider {
    private Relay slider;
    private InvertibleDigitalInput leftLimit;
    private InvertibleDigitalInput rightLimit;

    private int stateLeft;
    private boolean reachedLimitLeft;
    private boolean centeredLeft;
    private boolean onceLeft;
    private double timeToCenterLeft = 5;
    private double timeLeft;

    private int stateRight;
    private boolean reachedLimitRight;
    private boolean centeredRight;
    private boolean onceRight;
    private double timeToCenterRight = 5;
    private double timeRight;

    private boolean rightOnce;
    private boolean leftOnce;

    // RIGHT AND LEFT ARE BACKWARDS IN ALL OF THIS - IE LEFTLIMIT IS IN REALITY THE
    // RIGHT
    public Slider(Relay eSlider, InvertibleDigitalInput eLeftLimit, InvertibleDigitalInput eRightLimit) {
        slider = eSlider;
        leftLimit = eLeftLimit;
        rightLimit = eRightLimit;
        stateLeft = 0;
        reachedLimitLeft = false;
        centeredLeft = false;
        onceLeft = false;
    }

    public void init() {
        slider.set(Value.kOff);
    }

    public void update() {
        // Debug Values
        SmartDashboard.putBoolean("LeftLImit", leftLimit.get());
        SmartDashboard.putBoolean("RightLImit", rightLimit.get());
        SmartDashboard.putNumber("CoJoyX", JS_IO.coJoystick.getX());

        // Actions
        // ------------------------ Slide Operate ----------------------------
        if (JS_IO.coJoystick.getX() >= 0.15) { // If Co-Driver tilts to the left
            if (leftLimit.get()) { // If the motor reaches the right limit
                slider.set(Value.kOff); // Do nothing
            } else { // If not on the right limit
                slider.set(Value.kForward); // Adjust to the right
            }
        } else if (JS_IO.coJoystick.getX() < -0.15) { // If the Co-Driver tilts to the right
            if (rightLimit.get()) { // If the motor reaches the left limit
                slider.set(Value.kOff); // Do nothing
            } else { // If not on the left limit
                slider.set(Value.kReverse); // Adjust to the left
            }
        } else {
            slider.set(Value.kOff); // Do Nothing
        }

        // ------------------------ Slide Reset ----------------------------
        // Go Left ------------------------------------------------
        if (JS_IO.slideResetLeft.onButtonPressed()) { // The first time the Slide Reset Button is pressed
            reachedLimitLeft = false; // It hasnt reached the limit switch
            centeredLeft = false; // It isn't cenetered
            onceLeft = true; // Grab the time
            leftOnce = true;
        }
        if (JS_IO.slideResetLeft.isDown()) { // While the button is being pressed
            if (onceLeft) { // Grab the time
                timeLeft = Time.getTime();
                onceLeft = false;
            }

            if (!leftLimit.get() && !reachedLimitLeft && !centeredLeft) {
                if (leftOnce) {
                    stateLeft = 1;
                    leftOnce = false;
                }
            } else if (leftLimit.get() && !centeredLeft) {
                stateLeft = 2;
            }

            switch (stateLeft) {
                case 0: // Default
                    slider.set(Value.kOff);
                    break;
                case 1: // Move to limit switch
                    if (!leftLimit.get()) {
                        slider.set(Value.kForward);
                        reachedLimitLeft = false;
                    } else {
                        reachedLimitLeft = true;
                        onceLeft = true;
                        stateLeft = 2;
                    }
                    break;
                case 2:
                    slider.set(Value.kReverse);
                    if (((Time.getTime() - timeLeft) >= timeToCenterLeft) || rightLimit.get()) {
                        centeredLeft = true;
                        stateLeft = 0;
                    }
                    break;
            }
        }
        // Go Right ---------------------------------------------------------
        if (JS_IO.slideResetRight.onButtonPressed()) { // The first time the Slide Reset Button is pressed
            reachedLimitRight = rightLimit.get(); // It hasnt reached the limit switch
            centeredRight = false; // It isn't cenetered
            onceRight = true; // Grab the time
            rightOnce = true;
        }
        if (JS_IO.slideResetRight.isDown()) { // While the button is being pressed
            if (onceRight) { // Grab the time
                timeRight = Time.getTime();
                onceRight = false;
            }

            if (!rightLimit.get() && !reachedLimitRight && !centeredRight) {
                if (rightOnce) {
                    stateRight = 1;
                    rightOnce = false;
                }
            } else if (rightLimit.get() && !centeredRight) {
                stateRight = 2;
            }

            switch (stateRight) {
                case 0: // Default
                    slider.set(Value.kOff);
                    break;
                case 1: // Move to limit switch
                    if (!rightLimit.get()) {
                        slider.set(Value.kReverse);
                        reachedLimitRight = false;
                    } else {
                        reachedLimitRight = true;
                        onceRight = true;
                        stateRight = 2;
                    }
                    break;
                case 2:
                    slider.set(Value.kForward);
                    if (((Time.getTime() - timeRight) >= timeToCenterRight) || leftLimit.get()) {
                        centeredRight = true;
                        stateRight = 0;
                    }
                    break;
            }
        }

    }
}
