package org.usfirst.frc.team6193.robot.lib;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.SpeedController;

public class MyRobotDrive extends RobotDrive_3CIM2Speed{

	private CANTalon m_leftCIM1, m_rightCIM1;
	public MyRobotDrive(CANTalon leftCIM1, SpeedController leftCIM2, SpeedController leftMiniCIM,
			CANTalon rightCIM1, SpeedController rIghtCIM2, SpeedController rightMiniCIM) {
		super(leftCIM1, leftCIM2, leftMiniCIM, rightCIM1, rIghtCIM2, rightMiniCIM);
		
		m_leftCIM1 = leftCIM1;
		m_rightCIM1 = rightCIM1;
		// TODO Auto-generated constructor stub
	}

	
	@Override
	public double getDrivelineSpeed() {
		double leftMotorSpeed = m_leftCIM1.getSpeed();
		double rightMotorSpeed = m_rightCIM1.getSpeed();
		m_drivelineSpeed = (leftMotorSpeed + rightMotorSpeed) / 2.0;
		return m_drivelineSpeed;
	}

}
