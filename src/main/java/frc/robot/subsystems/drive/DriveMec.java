package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.hwd_io.IO;
import frc.robot.io.joysticks.JS_IO;
import frc.robot.io.joysticks.util.Axis;
import frc.robot.subsystems.Apriltags;

public class DriveMec {
    // hdw defintions:
    private static MecanumDrive mecDrv = IO.mecDrv;

    // joystick:
    private static Axis jsX = JS_IO.axLeftX;
    private static Axis jsY = JS_IO.axLeftY;
    private static Axis jsRot = JS_IO.axRightX;

    // variables:
    private static int state; // Shooter state machine. 0=Off by pct, 1=On by velocity, RPM
    private static int blState;

    private static Rotation2d heading;

    public static boolean brakeHold = false;

    private static double fwdSpd;
    private static double rlSpd;
    private static double rotSpd;
    private static boolean isFieldRelative;
    private static boolean balanceHold = false;
    private static double dgrHold = -1.0;
    // private static double dbDgrHold;

    private static double max_pow = 1;

    public static int targetingID = 1;
    public static int placementID = 2;

    private static final ProfiledPIDController driveController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)); // I tried removing period for mabye faster copmuting
    private static double dp = 0.034; // PID values for drive controller
    private static double di = 0.06;
    private static double dd = 0.06;
    private static double dv = 0.5; // Velocity
    private static double da = 0.1; // Acceleration
    private static double dt = 0.07; // Tolerance

    private static final ProfiledPIDController thetaController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    private static double tp = 0.4 // PID values for drive controller
    private static double ti = 0.0;
    private static double td = 0.0;
    private static double tv = 0.2; // Velocity
    private static double ta = 0.1; // Acceleration
    private static double tt = 0.07; // Tolerance


    /**
     * Initialize Shooter stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        sdbInit();
        cmdUpdate(0.0, 0.0, 0.0, false);
        state = 0; // Start at state 0, robot oriented by default
        blState = 0;
        brakeHold = false;
        dgrHold = -1.0;
        balanceHold = false;
        // dbDgrHold = 2;
        
        driveController.setPID(dp, di, dd);
        driveController.setTolerance(dt);
        driveController.setConstraints(new TrapezoidProfile.Constraints(dv, da));

        thetaController.setPID(tp, ti, td);
        thetaController.setTolerance(tt);
        thetaController.setConstraints(new TrapezoidProfile.Constraints(tv, ta));
    }

    /**
     * Update Mecanum Drive. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {
        heading = new Rotation2d(Math.toRadians(IO.navX.getNormalizedAngle()));
        // Add code here to start state machine or override the sm sequence

        smUpdate();
        sdbUpdate();
    }

    /**
     * @param id    Apriltag to use
     * @param place Which placement area to go to
     * 
     *              Placements Areas:
     *              1 2 3
     *              ⍲ ▢ ⍲
     *              4 5 6
     *              ⍲ ▢ ⍲
     *              7 8 9
     *              ⍲ ⍲ ⍲
     *              ▢ ▢ ▢
     */
    public static int goToTarget(int id, int place) {
        // check what targets are seen
        // based off that, check if the current tag what we are focused on.
        // if it is, go to the placement area

        sdbUpdate();

        if (!Apriltags.currentResult.hasTargets()) return -1;
        Pose2d suppliedPose = Apriltags.getTarget2DPose(id);
        Pose2d currentPose = Apriltags.getRobot2DPose();
        //Pose2d currentPose = new Pose2d(Apriltags.m_field.getRobotPose().getX() - 16.485 / 2,
        //        Apriltags.m_field.getRobotPose().getY() - 8.193 / 2,
        //        new Rotation2d(Apriltags.m_field.getRobotPose().getRotation().getRadians()));


        
        driveController.setPID(dp, di, dd);
        driveController.setTolerance(dt);
        driveController.setConstraints(new TrapezoidProfile.Constraints(dv, da));

        thetaController.setPID(tp, ti, td);
        thetaController.setTolerance(tt);
        thetaController.setConstraints(new TrapezoidProfile.Constraints(tv, ta));
        
        System.out.println("pvalue" + driveController.getP());

        if (place == 1 || place == 4 || place == 7) {
            suppliedPose = suppliedPose
                    .plus(new Transform2d(new Translation2d(1.2, -0.5588), new Rotation2d(Math.PI)));
        } else if (place == 3 || place == 6 || place == 9) {
            suppliedPose = suppliedPose
                    .plus(new Transform2d(new Translation2d(1.2, +0.5588), new Rotation2d(Math.PI)));
        } else if (place == 2 || place == 5 || place == 8) {
            suppliedPose = suppliedPose.plus(new Transform2d(new Translation2d(1.2, 0), new Rotation2d(Math.PI)));
        }

        double drivePowerScalar = driveController.calculate(
                currentPose.getTranslation().getDistance(suppliedPose.getTranslation()), 0.0);
        double turnPower = thetaController.calculate(
                currentPose.getRotation().getRadians(), suppliedPose.getRotation().getRadians());

        if (driveController.atGoal())
            drivePowerScalar = 0.0;
        if (thetaController.atGoal())
            turnPower = 0.0;

        var drivePower = new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(suppliedPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(new Translation2d(drivePowerScalar, 0), new Rotation2d(0)))
                .getTranslation();

        

        cmdUpdate(MathUtil.clamp(drivePower.getX(), -max_pow, max_pow),-MathUtil.clamp(drivePower.getY(), -max_pow, max_pow), -MathUtil.clamp(turnPower, -max_pow, max_pow),true);
        SmartDashboard.putNumber("apriltaging/x", MathUtil.clamp(drivePower.getX(), -max_pow, max_pow));
        SmartDashboard.putNumber("apriltaging/y",-MathUtil.clamp(drivePower.getY(), -max_pow, max_pow));
        SmartDashboard.putNumber("apriltaging/theta", -MathUtil.clamp(turnPower, -max_pow, max_pow));
        
        //System.out.println(MathUtil.clamp(drivePower.getY(), -max_pow, max_pow));

        //Apriltags.m_field.setRobotPose(MathUtil.clamp(drivePower.getX(), -max_pow, max_pow),        MathUtil.clamp(drivePower.getY(), -max_pow, max_pow), new Rotation2d(MathUtil.clamp(turnPower, -max_pow, max_pow)));
        return -1;

    }

    // ----------------- Shooter statuses and misc.-----------------
    /**
     * Probably shouldn't use this bc the states can change. Use statuses.
     * 
     * @return - present state of Shooter state machine.
     */
    public static int getState() {
        return state;
    }

    /**
     * @return If the state machine is running, not idle.
     */
    public static boolean getStatus() {
        return state != 0; // This example says the sm is runing, not idle.
    }

    /**
     * State Machine Update
     */

    // Robot Orientation
    private static void smUpdate() {
        switch (state) {
            case 0: // Robot oriented?
                cmdUpdate(-jsY.get(), jsX.get(), jsRot.get(), false);
                break;
            case 1: // Field oriented
                cmdUpdate(-jsY.get(), jsX.get(), jsRot.get(), true);
                break;
            case 2:
                goToTarget(targetingID, placementID);

            default: // all off
                cmdUpdate(0.0, 0.0, 0.0, false);
                System.out.println("Bad DriveMec state: " + state);
                break;

        }
    }

    /**
     * Issue spd setting as rpmSP if isVelCmd true else as percent cmd.
     * 
     * @param fwdSpd - Speed command to move robot forward.
     * @param rlSpd  - Speed command to move robot right & left.
     * @param rotSpd - Speed command to rotate robot.
     * 
     */
    private static void cmdUpdate(double _fwdSpd, double _rlSpd, double _rotSpd, boolean _isFieldRelative) {
        fwdSpd = _fwdSpd;
        rlSpd = _rlSpd;
        rotSpd = _rotSpd;
        isFieldRelative = _isFieldRelative;
        // Check any safeties, mod passed cmds if needed.
        chkInput();

        try {
            // Send commands to hardware
            if (isFieldRelative) {
                mecDrv.driveCartesian(fwdSpd, rlSpd, rotSpd, heading);
            } else {
                mecDrv.driveCartesian(fwdSpd, rlSpd, rotSpd);
            }
        } catch (Exception e) {
            System.out.println("DriveMec.cmdUpdate() failed: " + e);
            System.out.println("fwdSpd: " + fwdSpd + " rlSpd: " + rlSpd + " rotSpd: " + rotSpd + " isFieldRelative: "
                    + isFieldRelative + " heading: " + heading);
        }

        // // Send commands to hardware
        // if (isFieldRelative) {
        // mecDrv.driveCartesian(fwdSpd, rlSpd, rotSpd, heading);
        // } else {
        // mecDrv.driveCartesian(fwdSpd, rlSpd, rotSpd);
        // }

    }

    // ------------------------- SDB Stuff --------------------------------------
    /** Initialize sdb */
    private static void sdbInit() {
        // Put stuff here on the sdb to be retrieved from the sdb later
        // SmartDashboard.putBoolean("ZZ_Template/Sumpthin", sumpthin.get());
        // SmartDashboard.putNumber("dbHold", dbDgrHold);

        SmartDashboard.putNumber("apriltaging/dp",dp);
        SmartDashboard.putNumber("apriltaging/di",di);
        SmartDashboard.putNumber("apriltaging/dd",dd);
        SmartDashboard.putNumber("apriltaging/dv",dv);
        SmartDashboard.putNumber("apriltaging/da",da);
        SmartDashboard.putNumber("apriltaging/dt",dt);

        SmartDashboard.putNumber("apriltaging/tp",tp);
        SmartDashboard.putNumber("apriltaging/ti",ti);
        SmartDashboard.putNumber("apriltaging/td",td);
        SmartDashboard.putNumber("apriltaging/tv",tv);
        SmartDashboard.putNumber("apriltaging/ta",ta);
        SmartDashboard.putNumber("apriltaging/tt",tt);

    }

    /** Update the Smartdashboard. */
    private static void sdbUpdate() {
        // Put stuff to retrieve from sdb here. Must have been initialized in sdbInit().
        // sumpthin = SmartDashboard.getBoolean("ZZ_Template/Sumpthin", sumpthin.get());

        // Put other stuff to be displayed here
        SmartDashboard.putNumber("state", state);
        SmartDashboard.putNumber("Drv/rlSpd", rlSpd);
        SmartDashboard.putNumber("jsy", jsY.get());
        SmartDashboard.putNumber("jsx", jsX.get());
        SmartDashboard.putNumber("jsrot", jsRot.get());
        SmartDashboard.putNumber("bal/pitch", IO.navX.getPitch());
        SmartDashboard.putNumber("bal/state", blState);
        // dbDgrHold = SmartDashboard.getNumber("dbHold", dbDgrHold);

        SmartDashboard.putNumber("Gyro", IO.navX.getNormalizedAngle());
        SmartDashboard.putBoolean("robot", isFieldRelative);

        dp = SmartDashboard.getNumber("apriltaging/dp",dp);
        di = SmartDashboard.getNumber("apriltaging/di",di);
        dd = SmartDashboard.getNumber("apriltaging/dd",dd);
        dv = SmartDashboard.getNumber("apriltaging/dv",dv);
        da = SmartDashboard.getNumber("apriltaging/da", da);
        dt = SmartDashboard.getNumber("apriltaging/dt",dt);

        tp = SmartDashboard.getNumber("apriltaging/tp",tp);
        ti = SmartDashboard.getNumber("apriltaging/ti",ti);
        td = SmartDashboard.getNumber("apriltaging/td",td);
        tv = SmartDashboard.getNumber("apriltaging/tv",tv);
        ta = SmartDashboard.getNumber("apriltaging/ta",ta);
        tt = SmartDashboard.getNumber("apriltaging/tt",tt);



    }

    private static void chkInput() {
        chkBalanceHold();
        chkdgrHold();
    }

    private static void chkdgrHold() {
        if (dgrHold != -1.0) {
            rotSpd = -1.0 / 75.0 * (IO.navX.getNormalizedTo180() - dgrHold);
            // rotSpd = rotSpd < 0.2 ? 0.0 : rotSpd;
        }
    }

    // Target is/isnot in frame - TgtInFrame
    // Target is in limit - TgtLockedOn

    private static void chkBalanceHold() {
        if (balanceHold) {
            rlSpd = 0.0;
            rotSpd = 0.0;
            // isFieldRelative = false;
            switch (blState) {
                case 0:
                    fwdSpd = 0.0;
                    dgrHold = 0.0;
                    blState++;
                    break;
                case 1:
                    fwdSpd = 0.4;
                    if (Math.abs(IO.navX.getPitch()) > 14)
                        blState++;
                    break;
                case 2:
                    // fwdSpd = -0.4;
                    fwdSpd = 1.0 / 40.0 * (IO.navX.getPitch() - 3.0);
                    // if (Math.abs(IO.navX.getPitch() - 3.0) < 4) blState++;
                    break;
                case 3:
                    fwdSpd = 0.0;
                    System.out.println("3");
                    // IO.backLeft.clearStickyFaults(blState);
                    // climbHold = false;
                    // brakeHold = true;
                    break;
                case 4:

            }
        }
    }
}