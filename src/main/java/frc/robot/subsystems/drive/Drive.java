package frc.robot.subsystems.drive;

import frc.robot.subsystems.drive.DriveMath.DriveType;
import frc.robot.io.hwd_io.IO;
import frc.robot.io.hwd_io.util.NavX;
import frc.robot.io.joysticks.JS_IO;
import frc.robot.util.Time;
import frc.robot.util.TurnTo;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Updatable;

public class Drive implements Updatable {

	private DriveMath drive;
	private NavX gyro;

	private boolean grabAngleOnce = false;
	private double holdAngle = 0;

	private double kP = .002;
	private double kI;
	private double kD = 0.03;
	private double kF;
	private double maxI;

	// Scale Driver Sensitivity
	private double scaleFactor = 0.7;

	private boolean once = true;

	private boolean isFieldOriented = true;
	private TalonSRX[] driveMotors;

	private double time;

	public Drive(TalonSRX[] eDriveMotors, NavX eGyro) {
		driveMotors = eDriveMotors;
		gyro = eGyro;
		drive = new DriveMath(driveMotors, gyro);
	}

	public void init() {
		drive.setPIDValues(kP, kI, kD, kF, maxI);
		isFieldOriented = false;
		time = 0;
		SmartDashboard.putNumber("P", .002);
		SmartDashboard.putNumber("I", 0);
		SmartDashboard.putNumber("D", .03);
		SmartDashboard.putNumber("F", .002);
		SmartDashboard.putNumber("maxI", 0.1);
	}

	public void update() {
		drive.setPIDValues(SmartDashboard.getNumber("P", .002), SmartDashboard.getNumber("I", 0),
				SmartDashboard.getNumber("D", .03), SmartDashboard.getNumber("F", 0),
				SmartDashboard.getNumber("maxI", .1));
		// Toggle Field Oriented
		/*
		 * if(JoystickIO.fieldOrientedBtn.onButtonPressed()){
		 * if(isFieldOriented){
		 * isFieldOriented = false;
		 * }else{
		 * isFieldOriented = true;
		 * }
		 * }
		 */
		if (JS_IO.fieldOrientedBtn.onButtonPressed()) {
			isFieldOriented = true;
		}
		if (JS_IO.robotOrientedBtn.onButtonPressed()) {
			isFieldOriented = false;
		}

		// Reset the Gyro
		if (JS_IO.btnGyroReset.onButtonPressed()) {
			once = true;
			IO.navX.reset();
		}

		// Scale the drive sensitivity down
		double scaledX = (scaleFactor) * (JS_IO.rightJoystick.getX());
		double scaledY = (-scaleFactor) * (JS_IO.rightJoystick.getY());
		double scaledRotate = (scaleFactor) * (JS_IO.leftJoystick.getX());

		// Add deadband to Scaled Rotate
		if (scaledRotate < 0.2 && scaledRotate > -0.2) {
			scaledRotate = 0;
		}

		// Degub Info
		SmartDashboard.putNumber("Gyro", gyro.getNormalizedAngle());
		SmartDashboard.putBoolean("robot", isFieldOriented);
		SmartDashboard.putNumber("scaled rotate", scaledRotate);
		SmartDashboard.putNumber("Angle Being Held", holdAngle);
		SmartDashboard.putBoolean("Grab Angle Once", grabAngleOnce);

		// -------------------------------------------------------------------------
		// ----------------------------- Main Controller ---------------------------
		// -------------------------------------------------------------------------
		// if(JoystickIO.holdAngle.onButtonPressed()){
		// time = Time.getTime();
		// }
		SmartDashboard.putNumber("drive time", time);
		if (JS_IO.holdAngle90.isDown()) {
			if (isFieldOriented) {
				drive.drive(DriveType.ROTATE_PID, scaledX, scaledY, 0, TurnTo.align(gyro.getNormalizedAngle(), true));
				grabAngleOnce = true;
			} else {
				// if(Time.getTime() - time >= .5){
				drive.setRobotPID90(true);
				drive.drive(DriveType.ROBOT_PID, scaledX, scaledY, 0, 0);
				// }else{
				// drive.drive(DriveType.ROTATE_PID, scaledX, scaledY, 0,
				// TurnTo.align(gyro.getNormalizedAngle()));
				// }
				grabAngleOnce = true;
			}

		} else if (JS_IO.holdAngle30.isDown()) {
			if (isFieldOriented) {
				drive.drive(DriveType.ROTATE_PID, scaledX, scaledY, 0, TurnTo.align(gyro.getNormalizedAngle(), false));
				grabAngleOnce = true;
			} else {
				drive.setRobotPID90(false);
				drive.drive(DriveType.ROBOT_PID, scaledX, scaledY, 0, 0);
				grabAngleOnce = true;
			}

		} else {
			if (isFieldOriented) {
				if (scaledRotate == 0 && once) {
					time = Time.getTime();
					once = false;
				}
				if (scaledRotate == 0 && (Time.getTime() - time) >= 0.25) {
					SmartDashboard.putNumber("inside", 1);
					SmartDashboard.putBoolean("Is Holding Angle", true);
					if (grabAngleOnce) {
						grabAngleOnce = false;
						holdAngle = gyro.getNormalizedAngle();
					}
					drive.drive(DriveType.ROTATE_PID, scaledX, scaledY, scaledRotate, holdAngle);
				} else if (scaledRotate == 0) {
					SmartDashboard.putNumber("inside", 2);
					grabAngleOnce = true;
					drive.drive(DriveType.STICK_FIELD, scaledX, scaledY, scaledRotate, gyro.getNormalizedAngle());
				} else {
					SmartDashboard.putNumber("inside", 3);
					grabAngleOnce = true;
					once = true;
					drive.drive(DriveType.STICK_FIELD, scaledX, scaledY, scaledRotate, gyro.getNormalizedAngle());
				}
			} else {
				grabAngleOnce = true;
				drive.drive(DriveType.STICK_FIELD, scaledX, scaledY, scaledRotate, 0);
			}
		}
	}

	public void drive(DriveType driveState, double x, double y, double rotation, double angle) {
		drive.drive(driveState, x, y, rotation, angle);
	}

}
