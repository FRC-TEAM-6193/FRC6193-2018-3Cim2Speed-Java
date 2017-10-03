package org.usfirst.frc.team6193.robot;

// TODO: Fix name of one CIM on driveline

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	
	public static final double GEAR_AUTOMATIC_DOWNSHIFT_SPEED = 50.0;
	public static final double GEAR_AUTOMATIC_UPSHIFT_SPEED = 2000.0;

	public static final int K_RIGHT_CIM_MOTORCTRL_1_CANID = 11;
	public static final int K_RIGHT_CIM_MOTORCTRL_2_CANID = 10;
	public static final int K_RIGHT_MINICIM_MOTORCTRL_CANID = 9;
	public static final int K_LEFT_CIM_MOTORCTRL_1_CANID = 3;
	public static final int K_LEFT_CIM_MOTORCTRL_2_CANID = 5;
	public static final int K_LEFT_MINICIM_MOTORCTRL_CANID = 6;
	public static final int K_ROPEClIMBER_MOTCTRL_CANID = 12;
	public static final int K_GEARAHNDLER_SPIN_MOTCTRL_CANID = 8;
	public static final int K_GEARAHNDLER_LIFT_MOTCTRL_CANID = 7;

}
