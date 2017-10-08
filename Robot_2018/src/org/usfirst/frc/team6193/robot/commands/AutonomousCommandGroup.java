package org.usfirst.frc.team6193.robot.commands;

import org.usfirst.frc.team6193.robot.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousCommandGroup extends CommandGroup {
	private int m_DIOAutoIndex = 0;
    public AutonomousCommandGroup() {

    }
	public int GetDIOAutoIndex(){
		int rtn = 0x0;
//		DigitalInput DIO1 = new DigitalInput(1);
//		DigitalInput DIO2 = new DigitalInput(2);
//		DigitalInput DIO3 = new DigitalInput(3);
//		DigitalInput DIO4 = new DigitalInput(4);
		rtn |= Robot.DIO1.get() ? 0x8: 0;
		rtn |= Robot.DIO2.get() ? 0x4: 0;
		rtn |= Robot.DIO3.get() ? 0x2: 0;
		rtn |= Robot.DIO4.get() ? 0x1: 0;
		return rtn;
	}
	public void AutonomousInit() {
		m_DIOAutoIndex = GetDIOAutoIndex();
		
		switch(m_DIOAutoIndex){
		case 0x0:
			
			// Calibration setting, Leave default to do nothing
			// DrivelineDriveCommand(Speed,Distance,Gear,Time);
			addSequential(new DrivelineDriveCommand(0.7, 0, 1, 2.0));
//			addSequential(new DrivelineDriveCommand(0.2, 0, 1, 4.0));
//			addSequential(new DrivelineDriveCommand(0.3, 0, 1, 4.0));
//			addSequential(new DrivelineDriveCommand(0.4, 0, 1, 3.0));
//			addSequential(new DrivelineDriveCommand(0.5, 0, 1, 3.0));
//			addSequential(new DrivelineDriveCommand(0.6, 0, 1, 3.0));
//			addSequential(new DrivelineDriveCommand(0.7, 0, 1, 2.0));
//			addSequential(new DrivelineDriveCommand(0.8, 0, 1, 2.0));
//			addSequential(new DrivelineDriveCommand(0.9, 0, 1, 2.0));
//			addSequential(new DrivelineDriveCommand(1.0, 0, 1, 2.0));
			break;
		case 0x1:
			addSequential(new DrivelineDriveCommand(0.8, 0, 1, 3.0));
			// Drive straight to cross line, Use if in center position
			break;
		case 0x2:
			// Drive straight to opposite end line and stop
			addSequential(new DrivelineDriveCommand(0.9, 0, 1, 3.0));
			break;
		case 0x3:
			// Drive straight, turn 45, drive to opposite line
			addSequential(new DrivelineDriveCommand(1.0, 0, 1, 3.0));
			break;
		case 0x4:
			// Drive straight, turn -45, drive to opposite line
			addSequential(new DrivelineDriveCommand(1.0, 50, 1, 0));
			addSequential(new DrivelineRotateCommand(90, 0.6,1, 5));
			
			break;
		case 0x5:
			// Drive straight, turn 360, reverse, drive 
			break;
		case 0x6:
			// Drive straight, turn 90, drive to block side
			break;
		case 0x7:
			// Drive straight, turn -90, drive to block side
			break;
		}
	}
}
