
package org.usfirst.frc.team6193.robot;

import org.usfirst.frc.team6193.robot.commands.AutonomousCommandGroup;
import org.usfirst.frc.team6193.robot.subsystems.DrivelineSubsystem;
import org.usfirst.frc.team6193.robot.subsystems.GearHandlerSubsystem;
import org.usfirst.frc.team6193.robot.subsystems.RopeClimberSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {


	public static OI oi;
	public static DrivelineSubsystem driveline;
	public static RopeClimberSubsystem ropeClimber;
	public static GearHandlerSubsystem gearHandler;
	public static DigitalInput DIO1 = new DigitalInput(1);
	public static DigitalInput DIO2 = new DigitalInput(2);
	public static DigitalInput DIO3 = new DigitalInput(3);
	public static DigitalInput DIO4 = new DigitalInput(4);
	AutonomousCommandGroup auto;
	public void free() {
		
	}
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		driveline = new DrivelineSubsystem();
		driveline.initDriveline(1, false, true); // Low gear, do not use minicims and manual gear shifting
		ropeClimber = new RopeClimberSubsystem();
		gearHandler = new GearHandlerSubsystem();
		auto = new AutonomousCommandGroup();
		oi = new OI();

	}
	@Override
	public void robotPeriodic() {
		SmartDashboard.putBoolean("DIO1", DIO1.get());
		SmartDashboard.putBoolean("DIO2", DIO2.get());
		SmartDashboard.putBoolean("DIO3", DIO3.get());
		SmartDashboard.putBoolean("DIO4", DIO4.get());
		SmartDashboard.putBoolean("DrivelineIsAutomatic", driveline.getRobotDrive().getIsGearAutomaticMode());
		SmartDashboard.putNumber("DrivelineGear", driveline.getRobotDrive().getGear());
		SmartDashboard.putNumber("DrivelineAngle", driveline.getDrivelineAngle());
		SmartDashboard.putNumber("DrivelineSpeed", driveline.getRobotDrive().getDrivelineSpeed());
		SmartDashboard.putBoolean("DrivelineUseMiniCIM", driveline.getRobotDrive().getUseMiniCIMs());
		SmartDashboard.putNumber("DrivelinePosition", driveline.getRobotDrive().getDrivelinePosition());
		
	}
	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {


		auto.AutonomousInit();
		auto.start();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {

	}
}
