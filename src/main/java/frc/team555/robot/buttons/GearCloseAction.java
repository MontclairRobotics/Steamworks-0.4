package frc.team555.robot.buttons;

import org.montclairrobotics.sprocket.control.ButtonAction;
import org.montclairrobotics.sprocket.motors.Motor;

import edu.wpi.first.wpilibj.DigitalInput;

public class GearCloseAction extends ButtonAction {
	
	private Motor m;
	private DigitalInput closeSwitch;
	
	public GearCloseAction(Motor m, DigitalInput limitSwitch) {
		this.m = m;
		this.closeSwitch = limitSwitch;
	}
	
	@Override
	public void onAction() {
		if(closeSwitch.get()) {
			m.set(0.0);
		} else {
			m.set(-1);
		}
	}

}
