package frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;

public class GearOpenState extends AutoState {
	
	private Gear g;
	
	public GearOpenState(Gear g) {
		this.g=g;
	}
	
	@Override
	public boolean userIsDone() {
		return (g.getRightOpen() && g.getLeftOpen()) || (timeInState() > 2.0);
	}
	
	@Override
	public void userStart()
	{
		Robot.gearMode=Robot.GEAR_MODE.MANUAL;
		g.open();
	}
	@Override
	public void userStop() {
		g.stop();
	}

}
