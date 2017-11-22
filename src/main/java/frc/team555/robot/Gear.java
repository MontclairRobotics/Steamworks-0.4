package frc.team555.robot;

import org.montclairrobotics.sprocket.frc.FRCMotor;

import edu.wpi.first.wpilibj.DigitalInput;

public class Gear {
	
	private FRCMotor m1;
	private FRCMotor m2;
	private DigitalInput open1Switch;
	private DigitalInput open2Switch;
	private DigitalInput close1Switch;
	private DigitalInput close2Switch;
	
	public double gearSpeed = 0.35;
	

	public Gear(FRCMotor m1,DigitalInput openSwitch,DigitalInput closeSwitch,FRCMotor m2,DigitalInput open2Switch,DigitalInput close2Switch)
	{
		this.m1=m1;
		this.m2 = m2;
		this.open1Switch=openSwitch;
		this.close1Switch=closeSwitch;
		this.open2Switch = open2Switch;
		this.close2Switch = close2Switch;
	}
	
	public void openLimit()
	{
		if(!getLeftOpen()) {
			m1.set(gearSpeed);;
		} else {
			m1.set(0.0);
		}
		
		if(!getRightOpen()) {
			m2.set(gearSpeed);
		} else {
			m2.set(0.0);
		}
	}
	public void closeLimit()
	{
		if(!getLeftClose()) {
			m1.set(-gearSpeed);
		} else {
			m1.set(0.0);
		}
		
		if(!getRightClose()) {
			m2.set(-gearSpeed);
		} else {
			m2.set(0.0);
		}
	}
	
	public void open()
	{
		m1.set(gearSpeed);
		m2.set(gearSpeed);
	}
	public void close()
	{
		m1.set(-gearSpeed);
		m2.set(-gearSpeed);
	}
	public void stop()
	{
		m1.set(0.0);
		m2.set(0.0);
	}
	//6,7   10,11
	public void open1() {
		m1.set(gearSpeed);
	}
	
	public void close1() {
		m1.set(-gearSpeed);
	}
	
	public void stop1() {
		m1.set(0.0);
	}
	
	public void open2() {
		m2.set(gearSpeed);
	}
	
	public void close2() {
		m2.set(-gearSpeed);
	}
	
	public void stop2() {
		m2.set(0.0);
	}
	
	public boolean getRightOpen() {
		return !open2Switch.get();
	}
	
	public boolean getLeftOpen() {
		return open1Switch.get();
	}
	
	public boolean getRightClose() {
		return !close2Switch.get();
	}
	
	public boolean getLeftClose() {
		return close1Switch.get();
	}
}
