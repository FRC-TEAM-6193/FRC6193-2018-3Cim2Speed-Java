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
	public static final double kGearAutoShiftTurnIndicator = 0.1;
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
	private double leftFiltered = 0;
	private double rightFiltered = 0;
	private boolean shiftFinished = false;
	private double shiftStartTime = 0.0;

	/**3CIM 2Speed gearbox robotdrive constructor </br>
	 * Left and Right SpeedControllers 1 and 2 are required. </br>
	 * MiniCIM SpeedControllers may be null 
	 * 
	 * @param leftCIM1 SpeedController
	 * @param leftCIM2 SpeedController
	 * @param leftMiniCIM SpeedController may be null
	 * @param rightCIM1 SpeedController
	 * @param rightCIM2 SpeedController
	 * @param rightMiniCIM SpeedController may be null
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
	/**
	 * 
	 * @return Abstract method to return the Driveline Speed </br>
	 * 
	 * This is usually done through a encoder connected to the driveline
	 */
	public abstract double getDrivelineSpeed();
	
	/**
	 * 
	 * @param CANID The CANID of the PCM, usually 0
	 * @param leftIndex Left gearbox double solenoid start index on PCM output drive  
	 * @param rightIndex Right gearbox double solenoid start index on PCM output drive
	 */
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
	 * 
	 * @param moveValue Move value from -1 to 1
	 * @param rotateValue Rotate value from -1 to 1 </br>
	 * 
	 * Note:</br>
	 * Right and left motor speeds were reversed from original to match wiring for green forward on CANTalons
	 * 
	 */
	public void arcadeDrive(double moveValue, double rotateValue) {
		double leftMotorSpeed;
		double rightMotorSpeed;

		moveValue = limit(moveValue);
		rotateValue = limit(rotateValue);

		// square the inputs (while preserving the sign) to increase fine control
		// while permitting full power
		moveValue = Math.copySign(moveValue * moveValue, moveValue);
		rotateValue = Math.copySign(rotateValue * rotateValue, rotateValue);
		
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
	/**
	 * 
	 * @param gear gear as integer of 1 for low gear and 2 for high gear</br>
	 * 
	 * Calling this repeatedly with the current gear will do nothing
	 */
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
	 * Simple algorithm to shift up at the high rpm of the low gear and </br>
	 * shift down when almost at a stop.</b>
	 * A time limit of 1/2 between shifts is also added.</br>
	 * 
	 * Note:</br>
	 * Turning causes shifts and this is undesirable. Fix for that is done where left right motor speeds are set.
	 * @return the selected gear
	 */
	private int getNewAutomaticGear() {

		double average; 
		average = getDrivelineSpeed();
		average = Math.abs(average);
		if((Timer.getFPGATimestamp() - m_lastShiftTime) < 0.5) {
			return getGear();
		}
		
		if (average <= RobotMap.GEAR_AUTOMATIC_DOWNSHIFT_SPEED) {
			return 1;
		} else if (average > RobotMap.GEAR_AUTOMATIC_UPSHIFT_SPEED) {
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
		

		int gear;
		double ratio = 0.025;  // Value sets the delay to 0.5 seconds
		if(getIsGearAutomaticMode()) {
			// Attempt to prevent shifting during a turn.
			if(Math.abs(leftOutput + rightOutput ) < kGearAutoShiftTurnIndicator) {
				// TODO: cause a ramp to new speed. Ratio is 3.68:1 which causes abrupt change during a shift
				// 2400 max speed at low gear switching to HI is a new speed of 8700 speed on raw encoder.
				// Shift point is 2400 which turns out to be input of 0.9 at low which is 0.24 speed of high gear
				// Ramp over 0.5 seconds from 0.025 to input value.
				gear = getGear();                                                     // Get the current gear
				if(getNewAutomaticGear() == 2 && gear == 1) {                         // Is this the start of a up shift
					leftFiltered = leftOutput / 3.68;                                 // Start at the same speed as low gear
					rightFiltered = rightOutput / 3.68;
					shiftFinished = false;                                            // Shift has started
					shiftStartTime = Timer.getFPGATimestamp();                        // Time the shift started
				}
				
				leftFiltered = leftFiltered + Math.copySign(ratio, leftOutput);       // Add a constant value with the correct sign    
				rightFiltered = rightFiltered + Math.copySign(ratio, rightOutput);
				if(!shiftFinished) {                                                  // Do this until shift is complete
					if(Timer.getFPGATimestamp() > shiftStartTime + 0.5) {             // 1/2 second shift time
						shiftFinished = true;                                         // Time expired so set shift complete
					}else {
						leftOutput = leftFiltered;                                    // Replace the outputs with the new value
						rightOutput = rightFiltered;
					}
				}

				setGear(getNewAutomaticGear());
			}
			
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
