package frc.team555.robot;

import org.montclairrobotics.sprocket.drive.DTTarget;
import org.montclairrobotics.sprocket.pipeline.Step;
import org.montclairrobotics.sprocket.utils.PID;

public class VisionStep implements Step<DTTarget>{

	private double goal;
	private Vision vision;
	private double turnP;
	private PID turnPID;
	private double minTurnError;
	private double turn;
	
	public VisionStep(double goal,Vision vision,double turnP, double turnI, double turnD,double minTurnError)
	{
		this.goal=goal;
		this.vision=vision;
		this.turnPID = new PID(turnP, turnI, turnD).setInput(vision).setTarget(goal);
		this.minTurnError=minTurnError;
	}
	
	
	@Override
	public DTTarget get(DTTarget arg0) {
			turn=turnPID.get();
			return new DTTarget(arg0.getDirection(),turn);
	}

}
