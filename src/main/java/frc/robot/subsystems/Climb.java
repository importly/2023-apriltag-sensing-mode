
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.hwd_io.util.InvertibleDigitalInput;
import frc.robot.io.joysticks.JS_IO;
import frc.robot.util.Time;

public class Climb {
    private TalonSRX backRoller;
    private TalonSRX frontRoller;
    private Relay backWheel;
    private frc.robot.io.hwd_io.util.InvertibleDigitalInput rearLimit;
    private InvertibleDigitalInput frontLimit;

    private double backRollerPower = 0.65;
    private double frontRollerPower = 0.70;
    private double frontRollerIdlePower = frontRollerPower - 0.25;
    private double manualRollerPower = -0.3;
    private double backRollerIdle = .3;

    private boolean once = true;
    private double time;
    private int stateLow = 0;
    private int stateHigh = 0;
    private boolean cycled;

    public Climb(TalonSRX frontRoller, TalonSRX backRoller, Relay backWheel, InvertibleDigitalInput eRearLimit,
            InvertibleDigitalInput eFrontLimit) {
        this.frontRoller = frontRoller;
        this.backRoller = backRoller;
        this.backWheel = backWheel;
        rearLimit = eRearLimit;
        frontLimit = eFrontLimit;
        stateLow = 0;
        stateHigh = 0;
        cycled = true;
        SmartDashboard.putNumber("climb 1a", 0.5);
        SmartDashboard.putNumber("climb 1b", 2.5);
        SmartDashboard.putNumber("climb 1c", 10);
        SmartDashboard.putNumber("climb 1d", 3);

        SmartDashboard.putNumber("climb 2a", 0.1);
        SmartDashboard.putNumber("climb 2d", 15);

        SmartDashboard.putNumber("BRP", .65);
        SmartDashboard.putNumber("FRP", .70);
        SmartDashboard.putNumber("IDLE POWER", .45);

    }

    // maybe make an update that constantly checks for a button press
    public void update() {
        if (JS_IO.resetClimb.onButtonPressed()) {
            stateHigh = 0;
            stateLow = 0;
        }
        backRollerPower = SmartDashboard.getNumber("BRP", .65);
        frontRollerPower = SmartDashboard.getNumber("FRP", .70);
        frontRollerIdlePower = SmartDashboard.getNumber("IDLE POWER", .45);

        SmartDashboard.putNumber("StateHigh", stateHigh);
        SmartDashboard.putNumber("StateLow", stateLow);
        if (once) {
            time = Time.getTime();
            once = false;
        }

        if (JS_IO.highClimb.onButtonPressed()) {
            if (cycled) {
                stateHigh = 1;
            }
            once = true;
        }

        if (JS_IO.highClimb.isDown()) { // --------------------------------------------------------------- High
            switch (stateHigh) {
                case 0: // at the beginning, apply some drive motor power forward a little
                    climbCancel();
                    cycled = true;
                    break;
                case 1:// Drop the back
                    backWheel.set(Value.kForward);
                    frontRoller.set(ControlMode.PercentOutput, frontRollerPower);
                    cycled = false;
                    if (Time.getTime() - time >= SmartDashboard.getNumber("climb 2a", 0.1)) {
                        once = true;
                        stateHigh = 2;
                    }
                    break;
                case 2:// Roll front rollers
                    backWheel.set(Value.kForward);
                    frontRoller.set(ControlMode.PercentOutput, frontRollerPower);
                    backRoller.set(ControlMode.PercentOutput, backRollerPower);
                    backWheel.set(Value.kForward);
                    if (frontLimit.get()) { // check for the front limit switch here
                        once = true;
                        stateHigh = 3;
                    }
                    break;
                case 3:// Lower front power
                    backWheel.set(Value.kForward);
                    frontRoller.set(ControlMode.PercentOutput, frontRollerIdlePower);
                    backRoller.set(ControlMode.PercentOutput, backRollerPower);
                    if (rearLimit.get()) {
                        once = true;
                        stateHigh = 4;
                    }
                    break;
                case 4://
                    backWheel.set(Value.kForward);
                    backRoller.set(ControlMode.PercentOutput, 0.1); // hold power
                    if (!rearLimit.get()) {
                        once = true;
                        stateHigh = 3;
                    }

                    if (Time.getTime() - time >= SmartDashboard.getNumber("climb 2d", 15)) {
                        once = true;
                        stateHigh = 0;
                    }

                    break;
            }
        } else if (JS_IO.fixFrontClimb.isDown() || JS_IO.fixRearClimb.isDown()) {
            if (JS_IO.fixFrontClimb.isDown()) {
                frontRoller.set(ControlMode.PercentOutput, manualRollerPower);
            }
            if (JS_IO.fixRearClimb.isDown()) {
                backRoller.set(ControlMode.PercentOutput, manualRollerPower);
                backWheel.set(Value.kReverse);
            }
            if (JS_IO.fixRearClimb.onButtonReleased()) {
                backWheel.set(Value.kOff);
            }
        } else {
            if (JS_IO.lowClimb.onButtonPressed()) {
                if (cycled) {
                    stateLow = 1;
                }
                once = true;
            }
            if (JS_IO.lowClimb.isDown()) { // --------------------------------------------------------------- Low
                switch (stateLow) {
                    case 0:
                        climbCancel();
                        cycled = true;
                        break;
                    case 1:// Roll back rollers
                        frontRoller.set(ControlMode.PercentOutput, frontRollerPower);
                        cycled = false;
                        if (Time.getTime() - time >= SmartDashboard.getNumber("climb 1a", 0.5)) {
                            once = true;
                            stateLow = 2;
                        }
                        break;
                    case 2:// Drop the front
                        backRoller.set(ControlMode.PercentOutput, backRollerPower);
                        backWheel.set(Value.kForward);
                        if (frontLimit.get()) {
                            once = true;
                            stateLow = 3;
                        }
                        break; // somewhere here might need a case 3 to set backrolleridle sooner
                    case 3:// Lower front power
                        frontRoller.set(ControlMode.PercentOutput, frontRollerIdlePower);
                        backRoller.set(ControlMode.PercentOutput, backRollerIdle);
                        if (Time.getTime() - time >= SmartDashboard.getNumber("climb 1c", 10)) {
                            once = true;
                            stateLow = 0;
                        }
                        break;
                }
            } else {
                climbCancel();
            }
        }
    }

    public void climbCancel() { // stops all climb processes
        backWheel.set(Value.kOff);
        frontRoller.set(ControlMode.PercentOutput, 0.0);
        backRoller.set(ControlMode.PercentOutput, 0.0);
        once = true;
    }
}
