package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.hwd_io.util.ISolenoid;
import frc.robot.io.hwd_io.util.InvertibleDigitalInput;
import frc.robot.io.joysticks.JS_IO;
import frc.robot.util.Time;

public class Snorfler {
    private VictorSPX motor;
    private InvertibleDigitalInput baller;
    private ISolenoid downUp;
    private ISolenoid pushPull;
    // private Solenoid downUp;
    // private Solenoid pushPull;

    private double motorSpeed = 0.7;

    private boolean once;
    private double time;
    private int state;
    public static boolean running;
    private boolean povOn;

    // public Snorfler(VictorSPX eMotor, InvertibleDigitalInput eBaller, Solenoid
    // eDownUp, Solenoid ePushPull) {
    public Snorfler(VictorSPX eMotor, InvertibleDigitalInput eBaller, ISolenoid eDownUp, ISolenoid ePushPull) {
        motor = eMotor;
        baller = eBaller;
        downUp = eDownUp;
        pushPull = ePushPull;
        once = true;
        state = 0;
        povOn = false;
    }

    public void init() {
        motor.set(ControlMode.PercentOutput, 0);
        downUp.set(false);
    }

    public void update() {
        SmartDashboard.putBoolean("Baller", baller.get());
        SmartDashboard.putNumber("Snorfler States", state);

        // ----------------- Buton Presses ------------------------
        // Main Button
        if (JS_IO.snorfler.onButtonPressed() && !Shaft.state()) {
            once = true;
            if (state != 1) {
                state = 1; // start snorfle search
            } else {
                state = 2; // is in snorfle search, go back up
            }
        }
        // Trigger Button
        /*
         * if(JoystickIO.fork.onButtonPressed() && baller.get()){
         * if(state == 0){
         * state = 4;
         * }
         * }
         */
        // ------Radial Button Override Switch For ejecting balls-----
        if (JS_IO.overrideRadial.get() == 0) {
            povOn = true;
            motorSpeed = -0.7;
        } else if (JS_IO.overrideRadial.get() == 180) {
            povOn = true;
            motorSpeed = 0.7;
        } else {
            povOn = false;
        }

        // -------------- Cature Time -------------------
        if (once) {
            time = Time.getTime();
            once = false;
        }

        // ------------- Main State Machine --------------
        switch (state) {
            case 0: // Default placement
                downUp.set(false); // Pivot keeps the snorfler up and in the robot
                running = false; // Nothing is running
                motor.set(ControlMode.PercentOutput, (povOn) ? motorSpeed : 0.1); // Motor power defaults to zero,
                                                                                  // otherwise its the POV override
                break;
            case 1: // This is the grab state
                running = true;
                motor.set(ControlMode.PercentOutput, (povOn) ? motorSpeed : 0.7); // Motor power defaults to zero,
                                                                                  // otherwise its the POV override
                downUp.set(true); // The snorfler is extended down pickup Cargo
                pushPull.set(true); // The snorfler is extended out of the robot for clearance
                if (baller.get()) { // When we see that we have a Cargo loaded
                    once = true; // Grab the time
                    state = 2; // Go the the lift state
                }
                break;
            case 2: // Lift state 1
                running = true;
                downUp.set(false); // Bring the snorfler up
                motor.set(ControlMode.PercentOutput, (povOn) ? motorSpeed : 0.7); // Motor power is 0.3, just to make
                                                                                  // sure the ball doesnt fall out
                pushPull.set(true); // Keep the snorfler out of the robot
                if (Time.getTime() - time >= .5) { // Wait half a second, This is due to time it takes to raise the
                                                   // snorfler
                    once = true; // Get the time
                    state = 3; // Go to the next step for lifting the snorfler
                }
                break;
            case 3: // Lift state 2
                running = true;
                downUp.set(false); // Keep the snorfler up
                motor.set(ControlMode.PercentOutput, (povOn) ? motorSpeed : 0.7); // Motor defaults to 0.1
                pushPull.set(false); // Pull the snorfler inside the robot
                state = 0; // Return to the default state
                break;
            case 4: // Eject the Cargo
                running = true;
                motor.set(ControlMode.PercentOutput, (povOn) ? motorSpeed : -0.5); // Motor defaults to -0.5 to spit out
                                                                                   // the ball
                if (baller.get()) { // Check if we still have the ball
                    once = true; // Grabs the time
                } else {
                    if (Time.getTime() - time >= 0.5) { // Wait half a second
                        state = 5; // Go to the next state
                    }
                }
                break;
            case 5: // Stops the Cargo ejection
                motor.set(ControlMode.PercentOutput, (povOn) ? motorSpeed : 0); // Motor defaults to 0
                break;
        }
    }

    public static boolean running() {
        return running;
    }
}
