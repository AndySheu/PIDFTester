
package org.usfirst.frc.team972.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Robot extends IterativeRobot {
	Joystick leftJoy = new Joystick(0);
	Joystick rightJoy = new Joystick(1);
	Joystick opJoy = new Joystick(2);
	CANTalon talon = new CANTalon(0);
	PIDController pidController = new PIDController(0, 0, 0, 0, talon, talon);
	double kP, kI, kD, kF, setpoint = 0.5;
	PIDFVariable pidVar = PIDFVariable.NONE;
	boolean button11LastPressed = false, button12LastPressed = false;

	public void teleopInit() {
		checkPIDValues();
	}

	public void teleopPeriodic() {
		checkPIDValues();
		pidController.setPID(kP, kI, kD, kF);
		SmartDashboard.putNumber("Error", pidController.getError());
		if (opJoy.getRawButton(1)) {
			pidController.setSetpoint(setpoint);
			pidController.enable();
		} else {
			pidController.disable();
			talon.set(0);
		}
	}

	public void checkPIDValues() {
		if (opJoy.getRawButton(7)) {
			pidVar = PIDFVariable.P;
		} else if (opJoy.getRawButton(8)) {
			pidVar = PIDFVariable.I;
		} else if (opJoy.getRawButton(9)) {
			pidVar = PIDFVariable.D;
		} else if (opJoy.getRawButton(10)) {
			pidVar = PIDFVariable.F;
		}

		if (opJoy.getRawButton(11)) {
			if (!button11LastPressed) {
				changePIDVar(true);
			}
			button11LastPressed = true;
		} else {
			button11LastPressed = false;
		}

		if (opJoy.getRawButton(12)) {
			if (!button12LastPressed) {
				changePIDVar(false);
			}
			button12LastPressed = true;
		} else {
			button12LastPressed = false;
		}
	}

	public void changePIDVar(boolean increase) {
		switch (pidVar) {
		case P:
			if (increase) {
				kP += 0.001;
			} else {
				kP -= 0.001;
			}
			break;
		case I:
			if (increase) {
				kI += 0.001;
			} else {
				kI -= 0.001;
			}
			break;
		case D:
			if (increase) {
				kD += 0.001;
			} else {
				kD -= 0.001;
			}
			break;
		case F:
			if (increase) {
				kF += 0.001;
			} else {
				kF -= 0.001;
			}
			break;
		case NONE:
			break;
		default:
			break;
		}
	}
}