package org.usfirst.frc.team6193.robot.subsystems;

import org.usfirst.frc.team6193.robot.OI;
import org.usfirst.frc.team6193.robot.RobotMap;
import org.usfirst.frc.team6193.robot.commands.DrivelineDefaultCommand;
import org.usfirst.frc.team6193.robot.lib.RobotDrive_3C2S;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DrivelineSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private CANTalon m_rightFrontMotor;
	private CANTalon m_rightRearMotor;
	private CANTalon m_rightTopMotor;
	private CANTalon m_leftFrontMotor;
	private CANTalon m_leftRearMotor;
	private CANTalon m_leftTopMotor;
	private RobotDrive_3C2S m_robotDrive;
	public DrivelineSubsystem() {
		m_rightFrontMotor = new CANTalon(RobotMap.K_RIGHT_FRONT_MOTOR_CANID);
		m_rightRearMotor = new CANTalon(RobotMap.K_RIGHT_REAR_MOTOR_CANID);
		m_rightTopMotor = new CANTalon(RobotMap.K_RIGHT_TOP_MOTOR_CANID);
		m_leftFrontMotor = new CANTalon(RobotMap.K_LEFT_FRONT_MOTOR_CANID);
		m_leftRearMotor = new CANTalon(RobotMap.K_LEFT_REAR_MOTOR_CANID);
		m_leftTopMotor = new CANTalon(RobotMap.K_LEFT_TOP_MOTOR_CANID);
		
		m_robotDrive = new RobotDrive_3C2S(m_leftFrontMotor, m_leftRearMotor, m_leftTopMotor, m_rightFrontMotor, m_rightRearMotor, m_rightTopMotor);
		
	}
	public void initDriveline() {
		m_robotDrive.setGear(1);
		m_robotDrive.setUseMiniCIMs(true);
		m_robotDrive.setIsGearAutomaticMode(true);
	}
	/**
	 * Default drive mode with Joysticks 
	 */
	public void Drive() {
		double x = OI.xbox1.getRawAxis(0);
    	double y = OI.xbox1.getRawAxis(1);
    	Drive(x,y);
	}

	/**
	 * Drive command for move and rotate. Used in Autonomous and joystick drive
	 * @param move
	 * @param rotate
	 */
	public void Drive(double move, double rotate) {
		m_robotDrive.arcadeDrive(move, rotate);
		
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DrivelineDefaultCommand());
    }
}

