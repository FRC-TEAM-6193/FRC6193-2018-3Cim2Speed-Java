package org.usfirst.frc.team6193.robot.lib;


import static java.util.Objects.requireNonNull;

import org.usfirst.frc.team6193.robot.RobotMap;
import org.usfirst.frc.team6193.robot.lib.RobotDrive_3C2S.MotorSide;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 * 
 * @author Darryl Greathouse
 * 
 * Class usage. 
 * Extend this class and add required methods ...
 *
 */
public abstract class RobotDrive_3CIM2Speed implements MotorSafety{

	public static final double kDefaultExpirationTime = 0.1;
	public static final double kDefaultSensitivity = 0.5;
	public static final double kDefaultMaxOutput = 1.0;
	protected double m_sensitivity;
	protected double m_maxOutput;
	protected MotorSafetyHelper m_safetyHelper;
	
	protected SpeedController m_leftCIMMotorController1;
	protected SpeedController m_leftCIMMotorController2;
	protected SpeedController m_leftMiniCIMMotorController1;
	protected SpeedController m_rightCIMMotorController1;
	protected SpeedController m_rightCIMMotorController2;
	protected SpeedController m_rightMiniCIMMotorController1;
	protected DoubleSolenoid m_leftGearSolenoid;
	protected DoubleSolenoid m_rightGearSolenoid;
	protected boolean m_useMiniCIMs = true;
	protected boolean m_gearAutomaticMode = true;
	protected int m_gear = 1;
	protected double m_lastShiftTime = 0;
	protected double m_drivelineSpeed = 0;
	
	/**
	 * 
	 * @param leftCIM1  Motor Controller for left CIM #1
	 * @param leftCIM2
	 * @param leftMiniCIM
	 * @param rightCIM1
	 * @param rightCIM2
	 * @param rightMiniCIM
	 * 
	 * 
	 */
	public RobotDrive_3CIM2Speed(SpeedController leftCIM1,SpeedController leftCIM2,SpeedController leftMiniCIM,SpeedController rightCIM1,SpeedController rightCIM2,SpeedController rightMiniCIM) {
		m_leftCIMMotorController1 = requireNonNull(leftCIM1, "leftCIM1 cannot be null");
		m_leftCIMMotorController2 = requireNonNull(leftCIM2, "leftCIM2 cannot be null");
		m_leftMiniCIMMotorController1 = leftMiniCIM;
		m_rightCIMMotorController1 = requireNonNull(rightCIM1, "rightCIM1 cannot be null");
		m_rightCIMMotorController2 = requireNonNull(rightCIM2, "rightCIM2 cannot be null");
		m_rightMiniCIMMotorController1 = rightMiniCIM;
		m_sensitivity = kDefaultSensitivity;
		m_maxOutput = kDefaultMaxOutput;
		setupMotorSafety();
		
	}
	public abstract double getDrivelineSpeed();
	
	public void initDoubleSolenoids(int CANID, int leftIndex, int rightIndex) {
		m_rightGearSolenoid = new DoubleSolenoid(CANID,rightIndex,rightIndex + 1);
		m_leftGearSolenoid = new DoubleSolenoid(CANID,leftIndex,leftIndex + 1);
	}
	
	/**
	 * Invert a motor direction. This is used when a motor should run in the
	 * opposite direction as the drive code would normally run it. Motors that are
	 * direct drive would be inverted, the drive code assumes that the motors are
	 * geared with one reversal.
	 *
	 * @param motor
	 *            The motor index to invert.
	 * @param isInverted
	 *            True if the motor should be inverted when operated.
	 */
	public void setInvertedMotor(MotorSide motor, boolean isInverted) {
		switch (motor) {
		case KLEFT:
			m_leftCIMMotorController1.setInverted(isInverted);
			m_leftCIMMotorController2.setInverted(isInverted);
			if(m_leftMiniCIMMotorController1 != null) {
				m_leftMiniCIMMotorController1.setInverted(isInverted);
			}
			break;
		case KRIGHT:
			m_rightCIMMotorController1.setInverted(isInverted);
			m_rightCIMMotorController2.setInverted(isInverted);
			if(m_rightMiniCIMMotorController1 != null) {
				m_rightMiniCIMMotorController1.setInverted(isInverted);
			}
			break;
		default:
			throw new IllegalArgumentException("Illegal motor type: " + motor);
		}
	}
	
	/**
	 * Set the turning sensitivity.
	 *
	 * <p>
	 * This only impacts the drive() entry-point.
	 *
	 * @param sensitivity
	 *            Effectively sets the turning sensitivity (or turn radius for a
	 *            given value)
	 */
	public void setSensitivity(double sensitivity) {
		m_sensitivity = sensitivity;
	}

	/**
	 * Configure the scaling factor for using RobotDrive with motor controllers in a
	 * mode other than PercentVbus.
	 *
	 * @param maxOutput
	 *            Multiplied with the output percentage computed by the drive
	 *            functions.
	 */
	public void setMaxOutput(double maxOutput) {
		m_maxOutput = maxOutput;
	}
	
	/**
	 * Drive the motors at "outputMagnitude" and "curve". Both outputMagnitude and
	 * curve are -1.0 to +1.0 values, where 0.0 represents stopped and not turning.
	 * {@literal curve < 0 will turn left
	 * and curve > 0} will turn right.
	 *
	 * <p>
	 * The algorithm for steering provides a constant turn radius for any normal
	 * speed range, both forward and backward. Increasing sensitivity causes sharper
	 * turns for fixed values of curve.
	 *
	 * <p>
	 * This function will most likely be used in an autonomous routine.
	 *
	 * @param outputMagnitude
	 *            The speed setting for the outside wheel in a turn, forward or
	 *            backwards, +1 to -1.
	 * @param curve
	 *            The rate of turn, constant for different forward speeds. Set
	 *            {@literal
	 *                        curve < 0 for left turn or curve > 0 for right turn.}
	 *            Set curve = e^(-r/w) to get a turn radius r for wheelbase w of
	 *            your robot. Conversely, turn radius r = -ln(curve)*w for a given
	 *            value of curve and wheelbase w.
	 */
	public void curveDrive(double outputMagnitude, double curve) {
		final double leftOutput;
		final double rightOutput;

		if (curve < 0) {
			double value = Math.log(-curve);
			double ratio = (value - m_sensitivity) / (value + m_sensitivity);
			if (ratio == 0) {
				ratio = .0000000001;
			}
			leftOutput = outputMagnitude / ratio;
			rightOutput = outputMagnitude;
		} else if (curve > 0) {
			double value = Math.log(curve);
			double ratio = (value - m_sensitivity) / (value + m_sensitivity);
			if (ratio == 0) {
				ratio = .0000000001;
			}
			leftOutput = outputMagnitude;
			rightOutput = outputMagnitude / ratio;
		} else {
			leftOutput = outputMagnitude;
			rightOutput = outputMagnitude;
		}
		setLeftRightMotorOutputs(leftOutput, rightOutput);
	}

	/**
	 * Possible convention change. What I want
	 * + Forward and - Reverse
	 * + Right Turn  - Left Turn
	 * Arcade drive implements single stick driving. This function lets you directly
	 * provide joystick values from any source.
	 *
	 * @param moveValue
	 *            The value to use for forwards/backwards
	 * @param rotateValue
	 *            The value to use for the rotate right/left
	 */
	public void arcadeDrive(double moveValue, double rotateValue) {
		double leftMotorSpeed;
		double rightMotorSpeed;

		moveValue = limit(moveValue);
		rotateValue = limit(rotateValue);

		// square the inputs (while preserving the sign) to increase fine control
		// while permitting full power
		moveValue = moveValue >= 0 ? Math.pow(moveValue, 2) : -Math.pow(moveValue, 2);
		rotateValue = rotateValue >= 0 ? Math.pow(rotateValue, 2) : -Math.pow(rotateValue, 2);
		
		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				rightMotorSpeed = moveValue - rotateValue;
				leftMotorSpeed = Math.max(moveValue, rotateValue);
			} else {
				rightMotorSpeed = Math.max(moveValue, -rotateValue);
				leftMotorSpeed = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				rightMotorSpeed = -Math.max(-moveValue, rotateValue);
				leftMotorSpeed = moveValue + rotateValue;
			} else {
				rightMotorSpeed = moveValue - rotateValue;
				leftMotorSpeed = -Math.max(-moveValue, -rotateValue);
			}
		}

		setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
	}

	/**
	 * Limit motor values to the -1.0 to +1.0 range.
	 */
	protected static double limit(double num) {

		if (num > 1.0) {
			return 1.0;
		}
		if (num < -1.0) {
			return -1.0;
		}
		return num;
	}

	/**
	 * Initialize the gears to 1st gear
	 * Drive the pnuematic solenoid in one direction
	 */
	public void initGear(int gear) {
		if(gear == 1) {
			m_leftGearSolenoid.set(DoubleSolenoid.Value.kForward);
			m_rightGearSolenoid.set(DoubleSolenoid.Value.kForward);
			m_gear = 1;
		}else {
			m_leftGearSolenoid.set(DoubleSolenoid.Value.kReverse);
			m_rightGearSolenoid.set(DoubleSolenoid.Value.kReverse);
			m_gear = 2;
		}
	}
	public int getGear() {
		return m_gear;
	}

	public void setGear(int gear) {
		if(m_gear == 1 && gear == 2) {
			m_leftGearSolenoid.set(DoubleSolenoid.Value.kReverse);
			m_rightGearSolenoid.set(DoubleSolenoid.Value.kReverse);
			m_gear = 2;
			m_lastShiftTime = Timer.getFPGATimestamp();
		}if(m_gear == 2 && gear == 1) {
			m_leftGearSolenoid.set(DoubleSolenoid.Value.kForward);
			m_rightGearSolenoid.set(DoubleSolenoid.Value.kForward);
			m_gear = 1;
			m_lastShiftTime = Timer.getFPGATimestamp();
		}else if(m_gear == 1 && gear == 1) {
			// Do nothing
		}else if(m_gear == 2 && gear == 2) {
			// Do nothing
		}

	}
	/**
	 * New Algorithm
	 * 1. Shift up at ~4000 rpm
	 * 2. Shift down when <50 or ~0 rpm
	 * 3. Don't shift if last shift was less than 1/2 second ago
	 * This will prevent changes while slowing down and 
	 * slowing down from high gear takes no current. Keeps it consistemt.
	 * 
	 * This needs TESTING and improvement
	 * Unsure on getSpeed vs getEncVelocity
	 * The speed limit needs to be tested and calibrated
	 * @return
	 */
	private int getNewAutomaticGear() {
		// The speeds should be directionless. If EncVelocity works better, get Absolute
		// value
		//double leftMotorSpeed = m_leftCIMMotor1.getSpeed();
		//double rightMotorSpeed = m_rightCIMMotor1.getSpeed();
		double average; //(leftMotorSpeed + rightMotorSpeed) / 2.0;
		average = getDrivelineSpeed();
		average = Math.abs(average);
		if((Timer.getFPGATimestamp() - m_lastShiftTime) < 0.5) {
			return getGear();
		}
		
		if (average <= RobotMap.GEAR_AUTOMATIC_LOW_VALUE) {
			return 1;
		} else if (average > RobotMap.GEAR_AUTOMATIC_UPSHIFT_VALUE) {
			return 2;
		}else {
			 return getGear();
		}

	}
	public boolean getUseMiniCIMs() {
		return m_useMiniCIMs;
	}
	public void setUseMiniCIMs(boolean val) {
		m_useMiniCIMs = val;
	}
	/**
	 * @return the m_gearAutomaticMode
	 */
	public boolean getIsGearAutomaticMode() {
		return m_gearAutomaticMode;
	}

	/**
	 * @param m_gearAutomaticMode the m_gearAutomaticMode to set
	 */
	public void setIsGearAutomaticMode(boolean gearAutomaticMode) {
		this.m_gearAutomaticMode = gearAutomaticMode;
	}
	
	/**
	 * Set the motor outputs based on left and right values. TODO: Multiply Mini CIM
	 * or Top motor by scale factor, since the speeds are different.
	 * 
	 * @param leftOutput
	 * @param rightOutput
	 */
	public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {

		if(getIsGearAutomaticMode()) {
			setGear(getNewAutomaticGear());
		}
	
		if (m_leftCIMMotorController1 != null) {
			m_leftCIMMotorController1.set(limit(leftOutput) * m_maxOutput);
		}
		if (m_leftCIMMotorController2 != null) {
			m_leftCIMMotorController2.set(limit(leftOutput) * m_maxOutput);
		}
		if (m_leftMiniCIMMotorController1 != null) {
			if(m_useMiniCIMs) {
				m_leftMiniCIMMotorController1.set(limit(leftOutput) * m_maxOutput);
			}else {
				m_leftMiniCIMMotorController1.set(0);
			}
		}
		if (m_rightCIMMotorController1 != null) {
			m_rightCIMMotorController1.set(-limit(rightOutput) * m_maxOutput);
		}
		if (m_rightCIMMotorController2 != null) {
			m_rightCIMMotorController2.set(-limit(rightOutput) * m_maxOutput);
		}
		if (m_rightMiniCIMMotorController1 != null) {
			if(m_useMiniCIMs) {
				m_rightMiniCIMMotorController1.set(-limit(rightOutput) * m_maxOutput);
			}else {
				m_rightMiniCIMMotorController1.set(0);
			}
		}

		if (m_safetyHelper != null) {
			m_safetyHelper.feed();
		}
	}
	
	
	// Motor Safety Helper
	@Override
	public void setExpiration(double timeout) {
		m_safetyHelper.setExpiration(timeout);
	}

	@Override
	public double getExpiration() {
		return m_safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		return m_safetyHelper.isAlive();
	}

	@Override
	public boolean isSafetyEnabled() {
		return m_safetyHelper.isSafetyEnabled();
	}
	@Override
	public void setSafetyEnabled(boolean enabled) {
		m_safetyHelper.setSafetyEnabled(enabled);
	}
	@Override
	public String getDescription() {
		return "Robot Drive";
	}
	@Override
	public void stopMotor() {
		 if (m_leftCIMMotorController1 != null) {
			 m_leftCIMMotorController1.set(0);
		 }
		 if (m_leftCIMMotorController2 != null) {
			 m_leftCIMMotorController2.set(0);
		 }
		 if (m_leftMiniCIMMotorController1 != null) {
			 m_leftMiniCIMMotorController1.set(0);
		 }
		 if (m_rightCIMMotorController1 != null) {
			 m_rightCIMMotorController1.set(0);
		 }
		 if (m_rightCIMMotorController2 != null) {
			 m_rightCIMMotorController2.set(0);
		 }
		 if (m_rightMiniCIMMotorController1 != null) {
			 m_rightMiniCIMMotorController1.set(0);
		 }
		if (m_safetyHelper != null) {
			m_safetyHelper.feed();
		}
	}

	private void setupMotorSafety() {
		m_safetyHelper = new MotorSafetyHelper(this);
		m_safetyHelper.setExpiration(kDefaultExpirationTime);
		m_safetyHelper.setSafetyEnabled(true);
	}

}
