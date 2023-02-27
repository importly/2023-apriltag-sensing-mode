package frc.robot.io.hwd_io;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.io.hwd_io.util.InvertibleDigitalInput;
import frc.robot.io.hwd_io.util.InvertibleSolenoid;
import frc.robot.io.hwd_io.util.ISolenoid;
import frc.robot.io.hwd_io.util.NavX;

public class IO {
    // pdp
    public static PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
    // drive
    // To be used with the original drive.java
    // public static TalonSRX frontLeft = new TalonSRX(1);
    // public static TalonSRX frontLeftInv = new TalonSRX(2);
    // public static TalonSRX backLeft = new TalonSRX(3);
    // public static TalonSRX backLeftInv = new TalonSRX(4);
    // public static TalonSRX frontRight = new TalonSRX(5);
    // public static TalonSRX frontRightInv = new TalonSRX(6);
    // public static TalonSRX backRight = new TalonSRX(7);
    // public static TalonSRX backRightInv = new TalonSRX(8);
    // public static TalonSRX[] driveMotors = { frontLeft, frontLeftInv, backLeft,
    // backLeftInv,
    // /* */frontRight, frontRightInv, backRight, backRightInv };
    // To be used with MecanumDrive
    public static WPI_TalonSRX frontLeft = new WPI_TalonSRX(1);
    public static WPI_TalonSRX frontLeftInv = new WPI_TalonSRX(2);
    public static WPI_TalonSRX backLeft = new WPI_TalonSRX(3);
    public static WPI_TalonSRX backLeftInv = new WPI_TalonSRX(4);
    public static WPI_TalonSRX frontRight = new WPI_TalonSRX(5);
    public static WPI_TalonSRX frontRightInv = new WPI_TalonSRX(6);
    public static WPI_TalonSRX backRight = new WPI_TalonSRX(7);
    public static WPI_TalonSRX backRightInv = new WPI_TalonSRX(8);
    public static WPI_TalonSRX[] driveMotors = { frontLeft, frontLeftInv, backLeft, backLeftInv,
            /*                                         */frontRight, frontRightInv, backRight, backRightInv };
    public static MecanumDrive mecDrv = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    // navX
    public static NavX navX = new NavX(SPI.Port.kMXP);
    // camera
    // public static UsbCamera camOne =
    // CameraServer.getServer().startAutomaticCapture(0);
    // public static UsbCamera camTwo =
    // CameraServer.getInstance().startAutomaticCapture(1);
    // air
    public static PneumaticsModuleType pneuCtlMod = PneumaticsModuleType.CTREPCM;
    public static Compressor compressor = new Compressor(12, pneuCtlMod);
    public static Relay compressorRelay = new Relay(0);
    // fork
    public static ISolenoid forkSole = new InvertibleSolenoid(12, pneuCtlMod, 0);
    public static ISolenoid extendSole = new InvertibleSolenoid(12, pneuCtlMod, 1);
    // public static Solenoid extendSole = new Solenoid(pneuCtlMod, 1);
    public static InvertibleDigitalInput hasHatch = new InvertibleDigitalInput(0, false);
    // snorfler
    public static ISolenoid snorfleSole = new InvertibleSolenoid(12, pneuCtlMod, 2);
    // public static Solenoid snorfleSole = new Solenoid(pneuCtlMod, 2);
    public static InvertibleDigitalInput snorflerBanner = new InvertibleDigitalInput(3, false);
    public static VictorSPX snorflerMotor = new VictorSPX(11);
    // climb
    public static TalonSRX frontClimb = new TalonSRX(9);
    public static TalonSRX backClimb = new TalonSRX(10);
    public static Relay backWheelClimb = new Relay(1);
    public static InvertibleDigitalInput rearClimbLimit = new InvertibleDigitalInput(1, true);
    public static InvertibleDigitalInput frontClimbLimit = new InvertibleDigitalInput(4, true); // see if this is true
                                                                                                // or false
    // slider
    /*
     * public static Relay slipMotor = new Relay(2);
     * // These may or may not be backwards
     * public static InvertibleDigitalInput leftLimit = new
     * InvertibleDigitalInput(2,true);
     * public static InvertibleDigitalInput rightLimit = new
     * InvertibleDigitalInput(1, true);
     */
    // shaft
    public static ISolenoid shaftSole = new InvertibleSolenoid(pneuCtlMod, 3);

    static {
        frontRight.setInverted(true);
        frontRightInv.setInverted(true);
        backRight.setInverted(true);
        backRightInv.setInverted(true);
        backRightInv.follow(backRight);
        backLeftInv.follow(backLeft);
        frontRightInv.follow(frontRight);
        frontLeftInv.follow(frontLeft);

        // for(TalonSRX motor : driveMotors){
        // motor.configNeutralDeadband(.15);
        // }
        for (WPI_TalonSRX motor : driveMotors) {
            motor.configNeutralDeadband(.15);
        }
        frontClimb.setNeutralMode(NeutralMode.Brake);
        backClimb.setNeutralMode(NeutralMode.Brake);
    }
}
