
package frc.robot.subsystems;

import frc.robot.io.joysticks.JS_IO;
import frc.robot.io.hwd_io.util.ISolenoid;

public class Shaft {
    private ISolenoid shaft;
    private static boolean set;
    private static boolean state;

    public Shaft(ISolenoid shaft) {
        this.shaft = shaft;
        state = false;
        set = false;
    }

    // Start with the Four Bar down
    public void init() {
        shaft.set(false);
    }

    public void update() {
        if (JS_IO.shaft.onButtonPressed() || set) { // If we press the toggle button or variable
            if (state) { // If the Four Bar is up
                shaft.set(false); // Bring the Four Bar down
                state = false; // Set the state to down
                set = false; // Turn off the toggle variable
            } else { // If the Four Bar is down
                shaft.set(true); // Bring the Four Bar Up
                state = true; // Set the state to up
                set = false; // Turn off the toggle variable
            }
        }
    }

    public static void setLow() {
        state = true; // Force the state to up
        set = true; // Force the toggle to occur
    }

    public static boolean state() {
        return state;
    }
}
