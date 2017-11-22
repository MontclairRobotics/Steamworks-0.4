package frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;

public class GearCloseState extends AutoState {
	
private Gear g;
	
	public GearCloseState(Gear g) {
		this.g=g;
	}
	
	@Override
	public boolean userIsDone() {
		return this.timeInState() > 1.0;
	}
	
	@Override
	public void userStart()
	{
		Robot.gearMode= Robot.GEAR_MODE.MANUAL;
		g.close();
	}
	@Override
	public void userStop() {
		g.stop();
		Robot.gearMode=Robot.GEAR_MODE.MANUAL;
	}
}
