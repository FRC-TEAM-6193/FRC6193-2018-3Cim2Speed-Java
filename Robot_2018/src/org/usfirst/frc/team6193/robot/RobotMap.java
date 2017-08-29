package org.usfirst.frc.team6193.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static final double GEAR_AUTOMATIC_LOW_VALUE = 50.0;
	public static final double GEAR_AUTOMATIC_UPSHIFT_VALUE = 4000.0;

	public static final int K_RIGHT_CIM_MOTORCTRL_1_CANID = 1;
	public static final int K_RIGHT_CIM_MOTORCTRL_2_CANID = 2;
	public static final int K_RIGHT_MINICIM_MOTORCTRL_CANID = 3;
	public static final int K_LEFT_CIM_MOTORCTRL_1_CANID = 4;
	public static final int K_LEFT_CIM_MOTORCTRL_2_CANID = 5;
	public static final int K_LEFT_MINICIM_MOTORCTRL_CANID = 6;
	public static final int K_ROPEClIMBER_MOTCTRL_CANID = 7;
	public static final int K_GEARAHNDLER_SPIN_MOTCTRL_CANID = 0;
	public static final int K_GEARAHNDLER_ROTATE_MOTCTRL_CANID = 0;

}
