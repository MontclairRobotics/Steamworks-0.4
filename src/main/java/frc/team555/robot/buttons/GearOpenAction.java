package frc.team555.robot.buttons;

import org.montclairrobotics.sprocket.control.ButtonAction;
import org.montclairrobotics.sprocket.motors.Motor;

import edu.wpi.first.wpilibj.DigitalInput;

public class GearOpenAction extends ButtonAction {
	
	private Motor m;
	private DigitalInput openSwitch;
	
	public GearOpenAction(Motor m, DigitalInput limitSwitch) {
		this.m = m;
		this.openSwitch = limitSwitch;
	}
	
	@Override
	public void onAction() {
		if(openSwitch.get()) {
			m.set(0.0);
		} else {
			m.set(1);
		}
	}

}
