package frc.robot.io.joysticks;
/*
Original Author: Joey & Anthony
Rewite Author: Jim Hofmann
History:
J&A - 11/6/2019 - Original Release
JCH - 11/6/2019 - Original rework
JCH - 2/13/2022 - Got rid of jsConfig num.  Use chooser
TODO: Exception for bad or unattached devices.
      Auto config based on attached devices and position?
      Add enum for jsID & BtnID?  Button(eLJS, eBtn6) or Button(eGP, eBtnA)
Desc: Reads joystick (gamePad) values.  Can be used for different stick configurations
    based on feedback from Smartdashboard.  Various feedbacks from a joystick are
    implemented in classes, Button, Axis & Pov.
    This version is using named joysticks to istantiate axis, buttons & axis
*/

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.joysticks.util.Axis;
import frc.robot.io.joysticks.util.Button;
import frc.robot.io.joysticks.util.Pov;

//TODO: ASSIGN BUTTON PORTS FOR EACH BUTTON INITIALIZED !!!

//Declares all joysticks, buttons, axis & pov's.
public class JS_IO {
    private static int jsConfig;
    private static String prvJSAssign;

    // Declare all possible Joysticks
    public static Joystick leftJoystick = new Joystick(0); // Left JS
    public static Joystick rightJoystick = new Joystick(1); // Right JS
    public static Joystick coJoystick = new Joystick(2); // Co-Dvr JS
    public static Joystick gamePad = new Joystick(3); // Normal mode only (not Dual Trigger mode)
    public static Joystick neoPad = new Joystick(4); // Nintendo pamepad

    // Drive
    public static Axis axLeftDrive = new Axis(); // Left Drive
    public static Axis axRightDrive = new Axis(); // Right Drive

    public static Axis axLeftY = new Axis(); // Left JS Y - Added for testing in Drive3
    public static Axis axLeftX = new Axis(); // Left JS X
    public static Axis axRightY = new Axis(); // Right JS Y
    public static Axis axRightX = new Axis(); // Right JS X
    public static Axis axCoDrvY = new Axis(); // Co Drvr JS Y
    public static Axis axCoDrvX = new Axis(); // Co Drvr JS X

    public static Button btnGyroReset = new Button(); // L 6
    public static Button holdAngle90 = new Button(); // L 7
    public static Button holdAngle30 = new Button(); // L 5
    public static Button holdAngle0 = new Button(); // L 9
    public static Button fieldOrientedBtn = new Button(); // L 8
    public static Button robotOrientedBtn = new Button(); // l 10
    public static Button balanceBtn = new Button(); // l 11
    public static Button climbHoldBtn = new Button(); // l 12

    // Hatch
    public static Button fork = new Button(); // C 1
    // Climb
    public static Button lowClimb = new Button(); // C 12
    public static Button highClimb = new Button(); // C 10
    public static Button fixFrontClimb = new Button(); // C 7
    public static Button fixRearClimb = new Button(); // C 8
    public static Button resetClimb = new Button(); // C 2

    // Snorfler
    public static Button snorfler = new Button(); // C 4
    // Shaft
    public static Button shaft = new Button(); // C 11
    // Slide
    public static Button slideResetRight = new Button(); // C 5
    public static Button slideResetLeft = new Button(); // C 6
    // Overrides
    public static Pov overrideRadial = new Pov(); // C pov

    // test
    public static Button run = new Button(); // C 9

    // AprilTag Test
    public static Button goalHoldLFBtn = new Button();
    public static Button goalHoldLCBtn = new Button();
    public static Button goalHoldRFBtn = new Button();
    public static Button goalHoldRCBtn = new Button();

    // Constructor not needed, bc
    public JS_IO() {
        init();
    }

    public static void init() {
        chsrInit(); // Setup JS chooser and set JS assignments to default.
    }

    // ---- Joystick controller chooser ----
    private static SendableChooser<String> chsr = new SendableChooser<String>();
    private static final String[] chsrDesc = { "3-Joysticks", "2-Joysticks", "Gamepad", "Nintendo" };

    /** Setup the JS Chooser */
    public static void chsrInit() {
        for (int i = 0; i < chsrDesc.length; i++) {
            chsr.addOption(chsrDesc[i], chsrDesc[i]);
        }
        chsr.setDefaultOption(chsrDesc[0], chsrDesc[0]); // Chg index to select chsrDesc[] for default
        SmartDashboard.putData("JS/Choice", chsr);
        update(); // Update the JS assignments
    }

    public static void sdbUpdChsr() {
        SmartDashboard.putString("JS/Choosen", chsr.getSelected()); // Put selected on sdb
    }

    public static void update() { // Chk for Joystick configuration
        // System.out.println("Prv JS Assn: " + prvJSAssign + " =? "+
        // chsr.getSelected());
        if (prvJSAssign != (chsr.getSelected() == null ? chsrDesc[0] : chsr.getSelected())) {
            prvJSAssign = chsr.getSelected();
            sdbUpdChsr();
            caseDefault(); // Clear exisitng jsConfig
            System.out.println("JS Chsn: " + chsr.getSelected());
            configJS(); // then assign new jsConfig
        }
    }

    /** Configure a new JS assignment */
    public static void configJS() { // Configure JS controller assignments
        caseDefault(); // Clear exisitng jsConfig

        switch (prvJSAssign) { // then assign new assignments
            case "3-Joysticks": // Normal 3 joystick config
                norm3JS();
                break;
            case "2-Joysticks": // Normal 2 joystick config No CoDrvr
                norm2JS();
                break;
            case "Gamepad": // Gamepad only
                a_GP();
                break;
            case "Nintendo": // Nintendo only
                a_NP();
                break;
            default: // Bad assignment
                System.out.println("Bad JS choice - " + prvJSAssign);
                break;
        }
    }

    // ================ Controller actions ================

    // ----------- Normal 3 Joysticks -------------
    private static void norm3JS() {
        System.out.println("JS assigned to 3JS");

        // All stick axisesssss
        axLeftDrive.setAxis(leftJoystick, 1);
        axRightDrive.setAxis(rightJoystick, 1);

        axLeftX.setAxis(leftJoystick, 0); // Common call for each JS x & Y
        axLeftY.setAxis(leftJoystick, 1);
        axRightX.setAxis(rightJoystick, 0);
        axRightY.setAxis(rightJoystick, 1);
        axCoDrvX.setAxis(coJoystick, 0);
        axCoDrvY.setAxis(coJoystick, 1);

        // Drive buttons
        btnGyroReset.setButton(leftJoystick, 6);
        holdAngle90.setButton(leftJoystick, 7);
        holdAngle30.setButton(leftJoystick, 5);
        holdAngle0.setButton(leftJoystick, 9);
        fieldOrientedBtn.setButton(leftJoystick, 8);
        robotOrientedBtn.setButton(leftJoystick, 10);
        balanceBtn.setButton(leftJoystick, 11);
        climbHoldBtn.setButton(leftJoystick, 12);

        // Hatch
        fork.setButton(coJoystick, 1);

        // Climb
        lowClimb.setButton(coJoystick, 12);
        highClimb.setButton(coJoystick, 10);
        fixFrontClimb.setButton(coJoystick, 7);
        fixRearClimb.setButton(coJoystick, 8);
        resetClimb.setButton(coJoystick, 2);

        // Snorfler
        snorfler.setButton(coJoystick, 4);
        // Shaft
        shaft.setButton(coJoystick, 11);
        // Slide
        slideResetRight.setButton(coJoystick, 5);
        slideResetLeft.setButton(coJoystick, 6);
        // Overrides
        overrideRadial.setPov(coJoystick, 1);

        // test
        run.setButton(coJoystick, 9);

        // AprilTag Test
        goalHoldLFBtn.setButton(rightJoystick, 8);
        goalHoldLCBtn.setButton(rightJoystick, 9);
        goalHoldRFBtn.setButton(rightJoystick, 10);
        goalHoldRCBtn.setButton(rightJoystick, 11);
    }

    // ----- gamePad only --------
    // ----- gamePad only --------
    private static void a_GP() {
        System.out.println("JS assigned to GP");

        // All stick axisesssss
        axLeftDrive.setAxis(gamePad, 1);
        axRightDrive.setAxis(gamePad, 5);

        axLeftX.setAxis(gamePad, 0); // Added to test drive3
        axLeftY.setAxis(gamePad, 1);
        axRightX.setAxis(gamePad, 4);
        axRightY.setAxis(gamePad, 5);

        // Drive buttons
        btnGyroReset.setButton(gamePad, 8); // (back)
        holdAngle90.setButton(gamePad, 2); // (B)
        holdAngle0.setButton(gamePad, 4); // Y
        fieldOrientedBtn.setButton(gamePad, 1); // A
        robotOrientedBtn.setButton(gamePad, 3); // X

        // povTest.setPov(gamePad, 0);
    }

    // ----------- Normal 2 Joysticks -------------
    private static void norm2JS() {
    }

    // ----------- Nintendo gamepad -------------
    private static void a_NP() {
    }

    // ----------- Case Default -----------------
    private static void caseDefault() {
        // All stick axises
        axLeftDrive.setAxis(null, 0);
        axRightDrive.setAxis(null, 0);

        axLeftX.setAxis(null, 0);
        axLeftY.setAxis(null, 0);
        axRightX.setAxis(null, 0);
        axRightY.setAxis(null, 0);
        axCoDrvY.setAxis(null, 0); // Co Drvr JS Y
        axCoDrvX.setAxis(null, 0); // Co Drvr JS X

        // Drive buttons
        btnGyroReset.setButton(null, 0);
        holdAngle90.setButton(null, 0);
        holdAngle30.setButton(null, 0);
        holdAngle0.setButton(null, 0);
        fieldOrientedBtn.setButton(null, 0);
        robotOrientedBtn.setButton(null, 0);
        balanceBtn.setButton(null, 0);
        // Hatch
        fork.setButton(null, 0);

        // Climb
        lowClimb.setButton(null, 0);
        highClimb.setButton(null, 0);
        fixFrontClimb.setButton(null, 0);
        fixRearClimb.setButton(null, 0);
        resetClimb.setButton(null, 0);

        // Snorfler
        snorfler.setButton(null, 0);
        // Shaft
        shaft.setButton(null, 0);
        // Slide
        // slideResetRight.setButton(null, 0);
        // slideResetLeft.setButton(null, 0);
        // Overrides
        overrideRadial.setPov(null, 0);

        // test
        run.setButton(null, 0);
    }
}