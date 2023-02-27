package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.hwd_io.util.ISolenoid;
import frc.robot.io.hwd_io.util.InvertibleDigitalInput;
import frc.robot.io.joysticks.JS_IO;
import frc.robot.util.Time;

public class Fork {
    private frc.robot.io.hwd_io.util.ISolenoid forku;
    private ISolenoid shoot;
    private InvertibleDigitalInput hasHatch;

    private double time;
    private boolean once;
    private boolean firstPress;
    private boolean releaseOnce;
    private boolean notFive;

    private int state;

    public static boolean running;

    public Fork(ISolenoid forku, ISolenoid shoot, InvertibleDigitalInput hasHatch) {
        this.forku = forku;
        this.shoot = shoot;
        this.hasHatch = hasHatch;
        firstPress = true;
        releaseOnce = true;
        notFive = true;
        SmartDashboard.putNumber("fork 1", 0.5);
        SmartDashboard.putNumber("fork 2", 0.5);
        SmartDashboard.putNumber("fork 3", 0.5);
        SmartDashboard.putNumber("fork 4", 0.5);
        SmartDashboard.putNumber("fork 5", 0.5);
        SmartDashboard.putNumber("fork 6", 2);
        SmartDashboard.putNumber("fork 7", 0.5);
    }

    public void init(int estate, boolean auto) {
        state = estate;
        if (auto) {
            forku.set(false);
            shoot.set(false);
        }
    }

    public void update() {
        SmartDashboard.putBoolean("hastahc", hasHatch.get());
        SmartDashboard.putNumber("fork States", state);
        SmartDashboard.putBoolean("isdown", JS_IO.fork.isDown());
        if (JS_IO.fork.isDown()) {
            if (hasHatch.get() && !notFive) {
                state = 5;
            } else {
                if (firstPress) {
                    state = 1;
                    once = true;
                    firstPress = false;
                    releaseOnce = true;
                }
                notFive = true;
            }
        } else {
            if (releaseOnce) {
                once = true;
                state = 3;
                firstPress = true;
                releaseOnce = false;
                notFive = false;
            }

        }

        if (once) {
            time = Time.getTime();
            once = false;
        }

        switch (state) {
            case 0:
                running = false;
                forku.set(false);
                break;
            case 1:
                running = true;
                forku.set(true);
                shoot.set(false);
                if (Time.getTime() - time >= SmartDashboard.getNumber("fork 1", 0.5)) {
                    once = true;
                    state = 2;
                }
                break;
            case 2:
                running = true;
                forku.set(true);
                shoot.set(true);
                break;
            case 3:
                running = true;
                forku.set(false);
                shoot.set(true);
                if (Time.getTime() - time >= SmartDashboard.getNumber("fork 2", 0.5)) {
                    once = true;
                    state = 4;
                }
                break;
            case 4:
                running = true;
                forku.set(false);
                shoot.set(false);
                if (Time.getTime() - time >= SmartDashboard.getNumber("fork 3", 0.5)) {
                    once = true;
                    state = 0;
                }
                break;
            case 5: // Startt of Delivery
                running = true;
                forku.set(false);
                shoot.set(true);
                if (Time.getTime() - time >= SmartDashboard.getNumber("fork 4", 0.5)) {
                    once = true;
                    state = 6;
                }
                break;
            case 6:
                running = true;
                forku.set(true);
                shoot.set(true);
                if (Time.getTime() - time >= SmartDashboard.getNumber("fork 5", 0.5)) {
                    once = true;
                    state = 7;
                }
                break;
            case 7:
                running = true;
                forku.set(true);
                shoot.set(false);
                if (Time.getTime() - time >= SmartDashboard.getNumber("fork 6", 2)) {
                    once = true;
                    state = 8;
                }
                break;
            case 8:
                running = true;
                forku.set(false);
                shoot.set(false);
                if (Time.getTime() - time >= SmartDashboard.getNumber("fork 7", 0.5)) {
                    once = true;
                    state = 0;
                }
                break;

            /*
             * case 0:
             * forkSystem.setFork(false);
             * // forkSystem.setSlide(false);
             * break;
             * case 1: // holding out, ready to reccive hatch
             * forkSystem.setFork(true);
             * quickfix = true;
             * 
             * // if(!JoystickIO.deliverHatchBtn.isDown()){ once = true; forkState =
             * // 2; }
             * if(Time.getTime()-anthonytime >= .5){
             * forkState = 2;
             * }
             * break;
             * case 2:
             * forkSystem.setSlide(true);
             * quickfix = true;
             * 
             * break;
             * case 3:
             * forkSystem.setFork(false);
             * forkSystem.setSlide(true);
             * if (Time.getTime() - anthonytime >= .5) {
             * once = true;
             * quickfix = false;
             * forkState = 4;
             * }
             * break;
             * case 4:
             * forkSystem.setSlide(false);
             * forkState = 0;
             * break;
             * 
             * case 5:
             * forkSystem.setSlide(true);
             * if (Time.getTime() - anthonytime >= 1) {
             * once = true;
             * forkState = 6;
             * }
             * break;
             * case 6:
             * forkSystem.setFork(true);
             * forkSystem.setSlide(true);
             * if (Time.getTime() - anthonytime >= .5) {
             * once = true;
             * forkState = 7;
             * }
             * break;
             * case 7:
             * forkSystem.setSlide(false);
             * if(Time.getTime() - anthonytime >= 0.5){
             * once = true;
             * forkState = 8;
             * }
             * break;
             * case 8:
             * forkSystem.setFork(false);
             * forkState = 0;
             * break;
             */
        }

    }

    public int getState() {
        return state;
    }

    public static boolean running() {
        return running;
    }

}
