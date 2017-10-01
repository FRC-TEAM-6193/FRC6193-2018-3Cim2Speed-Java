package org.usfirst.frc.team6193.robot.subsystems;

import org.usfirst.frc.team6193.robot.RobotMap;
import org.usfirst.frc.team6193.robot.commands.GearHandlerDefaultCommand;
import org.usfirst.frc.team6193.robot.OI;
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GearHandlerSubsystem extends Subsystem {

	private CANTalon m_gearHandlerSpinMotor;
	private CANTalon m_gearHandlerLiftMotor;
	
	public GearHandlerSubsystem() {
		m_gearHandlerSpinMotor = new CANTalon(RobotMap.K_GEARAHNDLER_SPIN_MOTCTRL_CANID);
		m_gearHandlerLiftMotor = new CANTalon(RobotMap.K_GEARAHNDLER_LIFT_MOTCTRL_CANID);
	}
	public void HandlerGear() {
		
		if(OI.flight1.getRawButton(1)) {
			if(OI.flight1.getRawButton(9)) {
				m_gearHandlerSpinMotor.set(0.3); // Spinner Speed
			}else {
				m_gearHandlerSpinMotor.set(-0.4);
			}
			double X = OI.flight1.getX();
			if(X > 0.4) {
				X = 0.4;
			}else if(X < -0.4) {
				X = -0.4;
			}
			m_gearHandlerLiftMotor.set(X);
			SmartDashboard.putNumber("GearLift_X", X);
		}else {
			m_gearHandlerLiftMotor.set(0.0);
			m_gearHandlerSpinMotor.set(0.0);
		}
		
	}
    public void initDefaultCommand() {
        setDefaultCommand(new GearHandlerDefaultCommand());
    }
}

