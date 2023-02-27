// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.hwd_io.IO;
import frc.robot.io.joysticks.JS_IO;
import frc.robot.subsystems.Apriltags;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Fork;
import frc.robot.subsystems.Shaft;
import frc.robot.subsystems.Snorfler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveMec;
import frc.robot.util.Time;

/**
 * The VM is configured to automatically runs this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the project.
 */
public class Robot extends TimedRobot {
	Drive drive;
	Climb climb;
	// Fork fork;
	Shaft shaft;
	// // Slider slider;
	Snorfler snorfler;
	// // Stall stall;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		IO.navX.reset();
		drive = new Drive(IO.driveMotors, IO.navX);
		climb = new Climb(IO.frontClimb, IO.backClimb, IO.backWheelClimb, IO.rearClimbLimit, IO.frontClimbLimit);
		// fork = new Fork(IO.forkSole, IO.extendSole, IO.hasHatch);
		shaft = new Shaft(IO.shaftSole);
		// slider = new Slider(IO.slipMotor,IO.leftLimit,IO.rightLimit);
		snorfler = new Snorfler(IO.snorflerMotor, IO.snorflerBanner, IO.snorfleSole, IO.extendSole);
		// stall = new Stall();

		JS_IO.init(); // Init JS chooser & JS assignments.
		Apriltags.init(); // Init Apriltag
		DriveMec.init();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics that you want ran during disabled, autonomous, teleoperated
	 * and test.
	 */
	@Override
	public void robotPeriodic() {
		IO.compressor.enableDigital();
		IO.compressorRelay.set(IO.compressor.isEnabled() ? Relay.Value.kForward : Relay.Value.kOff);

		JS_IO.update();
		Apriltags.update();
		// Apriltags.calculateBestTarget();
	}

	/** This function is called oncce during autonomous. */
	@Override
	public void autonomousInit() {
		drive.init();

		// fork.init(0, true);
		shaft.init();
		// slider.init();
		snorfler.init();
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		IO.compressorRelay.set(IO.compressor.isEnabled() ? Relay.Value.kForward : Relay.Value.kOff);
		Time.update();
		JS_IO.update();

		//drive.update();

		climb.update();
		// fork.update();
		shaft.update();
		// slider.update();
		// snorfler.update();
		
		DriveMec.goToTarget(1, 2);
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		// drive.init();
		DriveMec.init();
		// fork.init(fork.getState(), false);
		shaft.init();
		// slider.init();
		snorfler.init();
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		// SmartDashboard.putBoolean("Rear Limit is inverted?",
		// IO.rearClimbLimit.get());
		// SmartDashboard.putBoolean("front Limit is inverted?",
		// IO.frontClimbLimit.get());
		// SmartDashboard.putNumber("Stall Talon Current",
		// IO.backClimb.getOutputCurrent());
		IO.compressorRelay.set(IO.compressor.enabled() ? Relay.Value.kForward : Relay.Value.kOff);
		Time.update();
		JS_IO.update();

		// drive.update();
		DriveMec.update();
		climb.update();
		// fork.update();
		// stall.update();
		shaft.update();
		//// slider.update();
		snorfler.update();

		if (JS_IO.run.isDown()) {
			// IO.backWheelClimb.set(Relay.Value.kForward);
		}

	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {
	}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
