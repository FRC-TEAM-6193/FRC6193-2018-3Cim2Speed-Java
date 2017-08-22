package org.usfirst.frc.team6193.robot;

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.hal.HAL;

/**
 * IterativeRobot implements the IterativeRobotBase robot program framework.
 *
 * <p>The IterativeRobot class is intended to be subclassed by a user creating a robot program.
 *
 * <p>periodic() functions from the base class are called each time a new packet is received from
 * the driver station.
 */
public class IterativeRobot extends IterativeRobotBase {
  public IterativeRobot() {
    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Iterative);
  }

  /**
   * Provide an alternate "main loop" via startCompetition().
   */
  public void startCompetition() {
	  boolean timeout;
	  int safetyCnt = 0;
    // loop forever, calling the appropriate mode-dependent function
    while (true) {
      // Wait for new data to arrive "NOT"
    	// Why wait for data from the Wifi, FMS and PC laptop. Run the loop every 20ms.
    	// Safety is the only reason. Possible to 
    	timeout = m_ds.waitForData(0.020); // returns true if data arrived and false if time elapsed.
    	// Options are to overdrive this or wait as a safety feature.
    	// This may be the reason 2016 robots were hitting the wall a few missed loops from network delay will cause the PID to not function correctly.
    	// We plan to use PIDs, so this needs to be fixed, if it is the problem.
    	// Safety feature
    	if(!timeout) {
    		safetyCnt++;
    	}else {
    		safetyCnt--;

    		if(safetyCnt < 0) {
    			safetyCnt = 0;
    		}
    	}
    	if(safetyCnt > 25) {
    		// Shut motors down.// set a flag for the robot.java to look at and kill motors
    		
    	}
      loopFunc();
    }
  }
}
