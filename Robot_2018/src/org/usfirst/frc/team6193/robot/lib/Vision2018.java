package org.usfirst.frc.team6193.robot.lib;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * 
 * @author darryl
 *Vision processing on RoboRIO. 
 *Other options are Raspberry PI through ethernet, MVIDIA or PC.
 *RoboRIO processing chosen for simplicity and algorithm works for dual scan and dual move.
 *
 *Integrate the Camerserver with a USB camera. 
 *Test a reflective tape cross as the target. 
 *Move the robot to point to the target.
 *Rotate the robot 360 and stop where the target is. This is for demos at sponsors.
 *Move to the within a couple feet of the target.
 *
 *1. Determine a student that want to learn and present this.
 *2. Code this up and test it.
 *3. Create a standing cross with the reflective tape.
 *
 */
public class Vision2018 {
	public Vision2018() {
		initVision();
	}
	public void initVision() {
		CameraServer.getInstance().startAutomaticCapture();
		CameraServer.getInstance().getVideo();
		CameraServer.getInstance().putVideo("Blur", 640, 480);
		UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
		
		MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
		mjpegServer1.setSource(usbCamera); CvSink cvSink = new CvSink("opencv_USB Camera 0");
		
		cvSink.setSource(usbCamera);
		CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
		MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
		mjpegServer2.setSource(outputStream);
		
		
	}
}
