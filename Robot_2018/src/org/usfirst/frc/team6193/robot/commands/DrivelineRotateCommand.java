package org.usfirst.frc.team6193.robot.commands;

import org.usfirst.frc.team6193.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */

public class DrivelineRotateCommand extends Command {
	private double m_Angle;
	private double m_StartAngle;
	private double m_Speed;
	private double m_Timeout;
	private int m_Gear;
    public DrivelineRotateCommand(double angle, double speed, int gear, double timeout) {
        requires(Robot.driveline);
        m_Angle = angle;
        m_Speed = speed;
        m_Gear = gear;
        m_Timeout = timeout;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveline.initDriveline(m_Gear,true,false);
    	m_StartAngle = Robot.driveline.getDrivelineAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveline.Drive(0, m_Speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	double currentAngle = Robot.driveline.getDrivelineAngle();
    	if(currentAngle > m_StartAngle + m_Angle || currentAngle < m_StartAngle - m_Angle || timeSinceInitialized()> m_Timeout){
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveline.Drive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
