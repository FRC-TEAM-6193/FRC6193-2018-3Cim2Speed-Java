package org.usfirst.frc.team6193.robot.lib;

import org.usfirst.frc.team6193.robot.Calibrations;


public class InterpolateGetTime {
	
	/**
	 * 
	 * @param distance Inches
	 * 
	 * @param DriveY Drive
	 * @return Seconds to travel the requested distance.
	 */
	public  double getTimeDrive(double distance,double speed, int gear){
		
		double inchPerSec = 0;
		if(gear == 1) {
			inchPerSec = LnrIntrpn(Calibrations.k_DrivelineSpeedTable_ULS, Calibrations.k_DrivelineSpeedLoGearTable_InchPerSec, Math.abs(speed));
		}else {
			inchPerSec = LnrIntrpn(Calibrations.k_DrivelineSpeedTable_ULS, Calibrations.k_DrivelineSpeedHiGearTable_InchPerSec, Math.abs(speed));
		}
		return (distance/inchPerSec);// * speed_sign;
	}

	private double LnrIntrpn(double[] XTbl, double[] YTbl, double Inp)
	{
		/* Local variables that only exist inside this method */
		int index;
		double Xdif;
		double Ydif;
		double XInpDif;
		
		/* Output variable */
		double Outp;
		
		/**/
		if(Inp <= XTbl[0]){
			Outp = XTbl[0];
		}else if(Inp >= XTbl[XTbl.length -1]){
			Outp = XTbl[XTbl.length -1];
		}else {
			index = 0;
			while(XTbl[index + 1] < Inp){
				index++;
			}
			Xdif = XTbl[index + 1] - XTbl[index];
			Ydif = YTbl[index + 1] - YTbl[index];
			XInpDif = Inp - XTbl[index];
			
			Outp = Ydif * XInpDif / Xdif + YTbl[index];
		}
		
		return Outp;
	}
	
}