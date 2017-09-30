package org.usfirst.frc.team6193.robot.lib;

import org.usfirst.frc.team6193.robot.Calibrations;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.SpeedController;

public class MyRobotDrive extends RobotDrive_3CIM2Speed{

	private CANTalon m_leftCIM1, m_rightCIM1;
	public MyRobotDrive(CANTalon leftCIM1, SpeedController leftCIM2, SpeedController leftMiniCIM,
			CANTalon rightCIM1, SpeedController rIghtCIM2, SpeedController rightMiniCIM) {
		super(leftCIM1, leftCIM2, leftMiniCIM, rightCIM1, rIghtCIM2, rightMiniCIM);
		
		m_leftCIM1 = leftCIM1;
		m_rightCIM1 = rightCIM1;
		
		initDoubleSolenoids(15, 0, 2);
		setGearboxShiftSpreadRatio(3.68);
		// TODO Auto-generated constructor stub
	}
	/**
	 * 
	 * @return The current position in inches.
	 */
	public double getDrivelinePosition() {
		return -m_leftCIM1.getEncPosition();
//		double leftPosition = m_leftCIM1.getEncPosition();
//		double rightPosition = m_rightCIM1.getEncPosition();
//		return ((leftPosition + rightPosition) /2.0) / Calibrations.k_DrivelineEncoderRatio_CntPerInch;
	}
	public void resetDrivelinePosition() {
		m_leftCIM1.setEncPosition(0);
		m_rightCIM1.setEncPosition(0);
	}
	@Override
	public double getDrivelineSpeed() {
		return -m_leftCIM1.getSpeed();
//		double leftMotorSpeed = m_leftCIM1.getSpeed();
//		double rightMotorSpeed = m_rightCIM1.getSpeed();
//		m_drivelineSpeed = (leftMotorSpeed + rightMotorSpeed) / 2.0;
//		return m_drivelineSpeed;
	}

}
