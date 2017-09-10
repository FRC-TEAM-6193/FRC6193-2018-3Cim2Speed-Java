package org.usfirst.frc.team6193.robot.commands;

import org.usfirst.frc.team6193.robot.lib.InterpolateGetTime;
import org.usfirst.frc.team6193.robot.Calibrations;
import org.usfirst.frc.team6193.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DrivelineDriveCommand extends Command {

	private double m_Distance;
	private double m_Speed;
	private InterpolateGetTime m_Interpolate;
	private double m_StartAngle;
	private int m_Gear;
	private double m_DriveTime;
	
	/**
	 * 
	 * @param speed The speed from -1.0 to 1.0
	 * @param distance The distance in inches to drive
	 * @param gear The gear to be used, 1 for low and 2 for high
	 * @param time The time to stop. Set to 0 if not calibrating a tables
	 */
    public DrivelineDriveCommand(double speed, double distance, int gear, double time) {
        requires(Robot.driveline);
        m_Interpolate = new InterpolateGetTime();
        m_Distance = distance;
        m_Speed = speed;
        m_Gear = gear;
        m_DriveTime = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveline.initDriveline(m_Gear,true,false);
    	m_StartAngle = Robot.driveline.getDrivelineAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double currentAngle = Robot.driveline.getDrivelineAngle();
    	double rotate  = -((currentAngle - m_StartAngle) * Calibrations.k_drivelineAutoDriveRotateComp);
    	Robot.driveline.Drive(m_Speed, rotate);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

    	if(m_DriveTime > 0.001) {
        	if(timeSinceInitialized() > m_DriveTime){
        		return true;
        	}
    	}else {
        	if(timeSinceInitialized() > m_Interpolate.getTimeDrive(m_Distance,m_Speed,m_Gear)){
        		return true;
        	}
    	}

        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveline.Drive(0.0, 0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
