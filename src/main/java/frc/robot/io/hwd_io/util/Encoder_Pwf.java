package frc.robot.io.hwd_io.util;

// import com.playingwithfusion.CANVenom;

// import edu.wpi.first.math.util.Units;

// public class Encoder_Pwf {

// private double tpf;
// private CANVenom venomCtlr;

// /**Interface to Venom motors with built-in encoders */
// public Encoder_Pwf(CANVenom escPort, double _tpf){
// tpf = _tpf;
// venomCtlr = escPort;
// }

// /**@return Encoder ticks. */
// public double ticks(){
// return venomCtlr.getPosition();
// }

// /**@return calculated feet from ticks. */
// public double feet(){
// // System.out.println(tpf);
// return tpf == 0.0 ? 0.0 : venomCtlr.getPosition() / tpf;
// }

// /**@return feet per second */
// public double getFPS(){
// return tpf == 0.0 ? 0.0 : 60 * (venomCtlr.getSpeed() / tpf); // (RPM / tpf =
// fpm) * 60 = fps
// }

// /**@return calcuate meters from feet. */
// public double meters() {
// return Units.feetToMeters(feet());
// }

// /** Reset encoder count to zero. */
// public void reset(){
// venomCtlr.resetPosition();
// // venomCtlr.setPosition(0.0);
// }

// /**
// * Added by Victor.
// * @return Signed RPM
// */
// public double getSpeed(){
// return venomCtlr.getSpeed();
// }

// /**@return the existiing ticks per foot, tpf. */
// public double getTPF() { return tpf; }

// /**@return the revolutions/ticks of the venom motor. */
// public double getTicks(){ return venomCtlr.getPosition(); }

// /**@param tpf - Set ticks per foot. */
// public void setTPF( double tpf) { this.tpf = tpf; }
// }
