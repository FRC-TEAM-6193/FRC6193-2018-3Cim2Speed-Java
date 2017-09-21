package org.usfirst.frc.team6193.robot;

import org.usfirst.frc.team6193.robot.commands.DrivelineUseMiniCIMsCommand;
import org.usfirst.frc.team6193.robot.commands.GearAutoModeCommand;
import org.usfirst.frc.team6193.robot.commands.GearSetHighGearCommand;
import org.usfirst.frc.team6193.robot.commands.GearSetLowGearCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public static Joystick xbox1 = new Joystick(0);
	public static Joystick flight1 = new Joystick(1);
	
	Button GearAutoModeBtn = new JoystickButton(xbox1, 2);
	Button GearSetHighBtn = new JoystickButton(xbox1, 4);
	Button GearSetLowBtn = new JoystickButton(xbox1, 1);
	Button DrivelineUseMiniCIMSbtn = new JoystickButton(xbox1, 3);
	public OI()
	{
		GearAutoModeBtn.whenPressed(new GearAutoModeCommand());
		GearSetHighBtn.whenPressed(new GearSetHighGearCommand());
		GearSetLowBtn.whenPressed(new GearSetLowGearCommand());
		DrivelineUseMiniCIMSbtn.whenPressed(new DrivelineUseMiniCIMsCommand());
	}
	
}
