package org.usfirst.frc.team6193.robot.subsystems;

import org.usfirst.frc.team6193.robot.Calibrations;
import org.usfirst.frc.team6193.robot.OI;
import org.usfirst.frc.team6193.robot.RobotMap;
import org.usfirst.frc.team6193.robot.commands.DrivelineDefaultCommand;
import org.usfirst.frc.team6193.robot.lib.MyRobotDrive;
import org.usfirst.frc.team6193.robot.lib.PIDMode;
//import org.usfirst.frc.team6193.robot.lib.RobotDrive_3C2S;
import org.usfirst.frc.team6193.robot.lib.RobotDrive_3C2S.MotorSide;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/** Driveline contains two gearboxes driving 3 wheels on each side. 
 *  The gearbox is a Dual speed ball shifter that can contain up to 3 full CIM motors.
 *  The plan is to have 2 Full CIMS and 1 MiniCIM on each gearbox.
 *  The robot drive system that controls the motors will also handle the shifting of the pneumatics solenoids.
 *  An encoder on the final stage of the gearbox gives a TBD ratio.
 *
 */
public class DrivelineSubsystem extends PIDSubsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private CANTalon m_rightCIMMotor1;
	private CANTalon m_rightCIMMotor2;
	private CANTalon m_rightMiniCIMMotor1;
	private CANTalon m_leftCIMMotor1;
	private CANTalon m_leftCIMMotor2;
	private CANTalon m_leftMiniCIMMotor1;
	private MyRobotDrive m_robotDrive;
	private ADXRS450_Gyro m_gyro;
	//private double m_position_inch;
	private PIDMode m_PIDMode = PIDMode.Position;
	
	
	public DrivelineSubsystem() {
		super(1,0,0);
		m_rightCIMMotor1 = new CANTalon(RobotMap.K_RIGHT_CIM_MOTORCTRL_1_CANID);
		m_rightCIMMotor2 = new CANTalon(RobotMap.K_RIGHT_CIM_MOTORCTRL_2_CANID);
		m_rightMiniCIMMotor1 = new CANTalon(RobotMap.K_RIGHT_MINICIM_MOTORCTRL_CANID);
		m_leftCIMMotor1 = new CANTalon(RobotMap.K_LEFT_CIM_MOTORCTRL_1_CANID);
		m_leftCIMMotor2 = new CANTalon(RobotMap.K_LEFT_CIM_MOTORCTRL_2_CANID);
		m_leftMiniCIMMotor1 = new CANTalon(RobotMap.K_LEFT_MINICIM_MOTORCTRL_CANID);
		m_gyro = new ADXRS450_Gyro();
		m_robotDrive = new MyRobotDrive(m_leftCIMMotor1, m_leftCIMMotor2, m_leftMiniCIMMotor1, m_rightCIMMotor1, m_rightCIMMotor2, m_rightMiniCIMMotor1);
		
	}
	/**
	 * 
	 * @param gear The gear to use, 1 = low and 2 = high
	 * @param useMiniCims Use the miniCim motor on the driveline
	 * @param useAutoMode True = Automatic and False = Manual
	 */
	public void initDriveline(int gear, boolean useMiniCims, boolean useAutoMode) {
		m_robotDrive.initGear(gear);
		m_robotDrive.setUseMiniCIMs(useMiniCims);
		m_robotDrive.setIsGearAutomaticMode(useAutoMode);
		m_robotDrive.setInvertedMotor(MotorSide.KRIGHT, true);
		m_robotDrive.resetDrivelinePosition();
	}
	/**
	 * Default drive mode with Joysticks 
	 */
	public void Drive() {
		
		// Thumbstick
		double rotate = OI.xbox1.getRawAxis(0);
		// Left Trigger Forward, Right Trigger Reverse
		double leftTrigger = OI.xbox1.getRawAxis(2);
		double rightTrigger = OI.xbox1.getRawAxis(3);
		double move = rightTrigger - leftTrigger;

    	Drive(move,rotate);
	}

	public MyRobotDrive getRobotDrive() {
		return m_robotDrive;
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
    public double getDrivelineAngle() {
    	return m_gyro.getAngle();
    }
	@Override
	protected double returnPIDInput() {
		if(m_PIDMode == PIDMode.Position) {
			return m_robotDrive.getDrivelinePosition();
		}else {
			return m_gyro.getAngle();
		}
	}
	@Override
	protected void usePIDOutput(double output) {
		if(m_PIDMode == PIDMode.Position) {
			Drive(output,0.0);
		}else {
			Drive(0.0,output);
		}
		
	}
	
}

