package frc.robot.ShamLib.motors;

//💪gains

public class PIDFGains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;

	public PIDFGains(double _kP, double _kI, double _kD, double _kF){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
	}
}
