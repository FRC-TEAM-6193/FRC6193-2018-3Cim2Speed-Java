package org.usfirst.frc.team6193.robot.lib;

/*----------------------------------------------------------------------------*/

/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



import static java.util.Objects.requireNonNull;

import org.usfirst.frc.team6193.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.CANTalon;

/**
 * Robot Drive class to handle a 3CIM motor Dual Speed gearbox on both sides of
 * robot. Since only 6 CIM motor can be used on a robot, this class only
 * supports 2 Full CIMs and one Mini CIM. Shifting is handled by a pneumatic
 * ball shifter when auto is selected and manually when in manual mode.
 * 
 * TODO: Manage the startup current of 6 CIMs.
 * Possibly have MiniCIM off from 0 to 500 RPM unless we are pushing something
 * 
 */
public class RobotDrive_3C2S implements MotorSafety {

	protected MotorSafetyHelper m_safetyHelper;

	/**
	 * The location of a motor on the robot for the purpose of driving.
	 */
	public enum MotorType {
		kLeftCIMMot1(0), kLeftCIMMot2(1), kLeftMiniCIM(2), kRightCIMMot1(3), kRightCIMMot2(4), kRightMiniCIM(5);
		public final int value;
		private MotorType(int value) {
			this.value = value;
		}
	}
	public enum MotorSide {
		KLEFT(0), KRIGHT(1);
		public final int value;
		private MotorSide (int value) {
			this.value = value;
		}
	}

	public static final double kDefaultExpirationTime = 0.1;
	public static final double kDefaultSensitivity = 0.5;
	public static final double kDefaultMaxOutput = 1.0;
	protected static final int kMaxNumberOfMotors = 6;
	protected double m_sensitivity;
	protected double m_maxOutput;
	protected CANTalon m_leftCIMMotor1;
	protected CANTalon m_leftCIMMotor2;
	protected CANTalon m_leftMiniCIMMotor1;
	protected CANTalon m_rightCIMMotor1;
	protected CANTalon m_rightCIMMotor2;
	protected CANTalon m_rightMiniCIMMotor1;
	protected static boolean kArcadeRatioCurve_Reported = false;
	protected static boolean kArcadeStandard_Reported = false;
	protected DoubleSolenoid m_leftGearSolenoid;
	protected DoubleSolenoid m_rightGearSolenoid;
	protected boolean m_useMiniCIMs = true;
	private boolean m_gearAutomaticMode = true;
	private int m_gear = 1;
	private double m_lastShiftTime = 0;
	private double m_drivelineSpeed = 0;
	

	/**
	 * Constructor for RobotDrive with 4 motors specified as SpeedController
	 * objects. Speed controller input version of RobotDrive (see previous
	 * comments).
	 *
	 * @param leftFrontMotor
	 *            The left front CANTalon object used to drive the robot (CIM)
	 * @param leftRearMotor
	 *            The left rear CANTalon object used to drive the robot. (CIM)
	 * @param leftTopMotor
	 *            The left top CANTalon object used to drive the robot. (Mini CIM)
	 * @param rightFrontMotor
	 *            The right front CANTalon object used to drive the robot. (CIM)
	 * @param rightRearMotor
	 *            The right rear CANTalon object used to drive the robot. (CIM)
	 * @param rightTopMotor
	 *            The right top CANTalon object used to drive the robot. (Mini CIM)
	 */
	public RobotDrive_3C2S(CANTalon leftFrontMotor, CANTalon leftRearMotor, CANTalon leftTopMotor,
			CANTalon rightFrontMotor, CANTalon rightRearMotor, CANTalon rightTopMotor) {
		m_leftCIMMotor1 = requireNonNull(leftFrontMotor, "leftFrontMotor cannot be null");
		m_leftCIMMotor2 = requireNonNull(leftRearMotor, "leftRearMotor cannot be null");
		m_leftMiniCIMMotor1 = requireNonNull(leftTopMotor, "leftTopMotor cannot be null"); // TODO: insure brake is off
		m_rightCIMMotor1 = requireNonNull(rightFrontMotor, "rightFrontMotor cannot be null");
		m_rightCIMMotor2 = requireNonNull(rightRearMotor, "rightRearMotor cannot be null");
		m_rightMiniCIMMotor1 = requireNonNull(rightTopMotor, "rightTopMotor cannot be null"); // TODO: insure brake is off
		m_rightGearSolenoid = new DoubleSolenoid(15,0,1);
		m_leftGearSolenoid = new DoubleSolenoid(15,2,3);
		m_leftMiniCIMMotor1.enableBrakeMode(false);
		m_rightMiniCIMMotor1.enableBrakeMode(false);
		m_sensitivity = kDefaultSensitivity;
		m_maxOutput = kDefaultMaxOutput;
		setupMotorSafety();
		drive(0, 0);
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
	public void drive(double outputMagnitude, double curve) {
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
		if (moveValue >= 0.0) {
			moveValue = moveValue * moveValue;
		} else {
			moveValue = -(moveValue * moveValue);
		}
		if (rotateValue >= 0.0) {
			rotateValue = rotateValue * rotateValue;
		} else {
			rotateValue = -(rotateValue * rotateValue);
		}

		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = Math.max(moveValue, rotateValue);
			} else {
				leftMotorSpeed = Math.max(moveValue, -rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				leftMotorSpeed = -Math.max(-moveValue, rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			} else {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
			}
		}

		setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
	}

	/**
	 * Set the motor outputs based on left and right values. TODO: Multiply Mini CIM
	 * or Top motor by scale factor, since the speeds are different.
	 * 
	 * @param leftOutput
	 * @param rightOutput
	 */
	public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
		if (m_rightCIMMotor2 == null || m_rightCIMMotor1 == null || m_rightMiniCIMMotor1 == null || m_leftCIMMotor1 == null
				|| m_leftCIMMotor2 == null || m_leftMiniCIMMotor1 == null) {
			throw new NullPointerException("Null motor provided");
		}
		if(getIsGearAutomaticMode()) {
			setGear(getNewAutomaticGear());
		}
	
		if (m_leftCIMMotor1 != null) {
			m_leftCIMMotor1.set(limit(leftOutput) * m_maxOutput);
		}
		if (m_leftCIMMotor2 != null) {
			m_leftCIMMotor2.set(limit(leftOutput) * m_maxOutput);
		}
		if (m_leftMiniCIMMotor1 != null) {
			if(m_useMiniCIMs) {
				m_leftMiniCIMMotor1.set(limit(leftOutput) * m_maxOutput);
			}else {
				m_leftMiniCIMMotor1.set(0);
			}
		}
		if (m_rightCIMMotor1 != null) {
			m_rightCIMMotor1.set(-limit(rightOutput) * m_maxOutput);
		}
		if (m_rightCIMMotor2 != null) {
			m_rightCIMMotor2.set(-limit(rightOutput) * m_maxOutput);
		}
		if (m_rightMiniCIMMotor1 != null) {
			if(m_useMiniCIMs) {
				m_rightMiniCIMMotor1.set(-limit(rightOutput) * m_maxOutput);
			}else {
				m_rightMiniCIMMotor1.set(0);
			}
		}

		if (m_safetyHelper != null) {
			m_safetyHelper.feed();
		}
	}
	public  double getSpeed() {
		return 0;
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
		double leftMotorSpeed = m_leftCIMMotor1.getSpeed();
		double rightMotorSpeed = m_rightCIMMotor1.getSpeed();
		double average = (leftMotorSpeed + rightMotorSpeed) / 2.0;
		
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
	 * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
	 */
	protected static void normalize(double[] wheelSpeeds) {
		double maxMagnitude = Math.abs(wheelSpeeds[0]);
		for (int i = 1; i < kMaxNumberOfMotors; i++) {
			double temp = Math.abs(wheelSpeeds[i]);
			if (maxMagnitude < temp) {
				maxMagnitude = temp;
			}
		}
		if (maxMagnitude > 1.0) {
			for (int i = 0; i < kMaxNumberOfMotors; i++) {
				wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
			}
		}
	}

	/**
	 * Rotate a vector in Cartesian space.
	 */
	protected static double[] rotateVector(double x, double y, double angle) {
		double cosA = Math.cos(angle * (3.14159 / 180.0));
		double sinA = Math.sin(angle * (3.14159 / 180.0));
		double[] out = new double[2];
		out[0] = x * cosA - y * sinA;
		out[1] = x * sinA + y * cosA;
		return out;
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
			m_leftCIMMotor1.setInverted(isInverted);
			m_leftCIMMotor2.setInverted(isInverted);
			m_leftMiniCIMMotor1.setInverted(isInverted);
			break;
		case KRIGHT:
			m_rightCIMMotor1.setInverted(isInverted);
			m_rightCIMMotor2.setInverted(isInverted);
			m_rightMiniCIMMotor1.setInverted(isInverted);
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
	 * Free the speed controllers if they were allocated locally.
	 */
	public void free() {

	}

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
		 if (m_leftCIMMotor1 != null) {
			 m_leftCIMMotor1.set(0);
		 }
		 if (m_leftCIMMotor2 != null) {
			 m_leftCIMMotor2.set(0);
		 }
		 if (m_leftMiniCIMMotor1 != null) {
			 m_leftMiniCIMMotor1.set(0);
		 }
		 if (m_rightCIMMotor1 != null) {
			 m_rightCIMMotor1.set(0);
		 }
		 if (m_rightCIMMotor2 != null) {
			 m_rightCIMMotor2.set(0);
		 }
		 if (m_rightMiniCIMMotor1 != null) {
			 m_rightMiniCIMMotor1.set(0);
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

	protected int getNumMotors() {
		int motors = 0;
		if (m_leftCIMMotor1 != null) {
			motors++;
		}
		if (m_leftCIMMotor2 != null) {
			motors++;
		}
		if (m_leftMiniCIMMotor1 != null) {
			motors++;
		}
		if (m_rightCIMMotor1 != null) {
			motors++;
		}
		if (m_rightCIMMotor2 != null) {
			motors++;
		}
		if (m_rightMiniCIMMotor1 != null) {
			motors++;
		}
		return motors;
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
	 * @return the m_drivelineSpeed
	 */
	public double getDrivelineSpeed() {
		double leftMotorSpeed = m_leftCIMMotor1.getSpeed();
		double rightMotorSpeed = m_rightCIMMotor1.getSpeed();
		m_drivelineSpeed = (leftMotorSpeed + rightMotorSpeed) / 2.0;
		return m_drivelineSpeed;
	}

}
