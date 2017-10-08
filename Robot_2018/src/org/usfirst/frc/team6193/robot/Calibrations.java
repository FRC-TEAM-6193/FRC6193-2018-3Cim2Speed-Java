package org.usfirst.frc.team6193.robot;

public class Calibrations {
	// These tables are used for Autonomous drive when we are not doing PID control. This allows a time control based on inches.
	public static double[] k_DrivelineSpeedTable_ULS = {0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};
	public static double[] k_DrivelineSpeedLoGearTable_InchPerSec = {10,10,10,10,10,10,10,29,39.8,49.3,61.6}; // TODO: Calibrate Inch/Sec Tables
	public static double[] k_DrivelineSpeedHiGearTable_InchPerSec = {0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};
	public static double k_DrivelineEncoderRatio_CntPerInch = 404;
	public static double k_drivelineAutoDriveRotateComp = 0.05;
	
}
