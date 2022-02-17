// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


//Single hook
//Double hook, firstHook, secondHook
public class Climber extends SubsystemBase {

	WPI_TalonFX climberRight = new WPI_TalonFX(Constants.CLIMBER_RIGHT_CAN_ID);
	WPI_TalonFX climberLeft = new WPI_TalonFX(Constants.CLIMBER_LEFT_CAN_ID);

	/** Creates a new Climber. */
	public Climber() {
		climberRight.configFactoryDefault();
		climberLeft.configFactoryDefault();
		climberLeft.follow(climberRight);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void spin(double speed) {
		climberRight.set(speed);
	}

	public void hold() {
		double climberPosition = climberRight.getSelectedSensorPosition();
		climberRight.set(ControlMode.Position, climberPosition);
	}

	public void stop() {
		climberRight.set(0);
	}

}
