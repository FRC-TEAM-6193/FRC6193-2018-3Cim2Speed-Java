package org.usfirst.frc.team6193.robot.subsystems;

import org.usfirst.frc.team6193.robot.OI;
import org.usfirst.frc.team6193.robot.RobotMap;
import org.usfirst.frc.team6193.robot.commands.RopeClimberDefaultCommand;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class RopeClimberSubsystem extends Subsystem {


	private CANTalon m_ropeClimberMotor;
	
	public RopeClimberSubsystem() {
		m_ropeClimberMotor = new CANTalon(RobotMap.K_ROPEClIMBER_MOTCTRL_CANID);
	}
	
	public void Climb() {
		double speed = 0;
		if(OI.flight1.getRawButton(12)) {
			double yAxis = OI.flight1.getY();
			speed = yAxis;
			if(m_ropeClimberMotor.getOutputCurrent() > 60.0) {
				speed = 0;
			}
		}
		m_ropeClimberMotor.set(Math.abs(speed));
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new RopeClimberDefaultCommand());
    }
}

