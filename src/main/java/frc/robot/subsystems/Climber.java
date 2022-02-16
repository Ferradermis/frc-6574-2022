// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


//Single hook
//Double hook, firstHook, secondHook
public class Climber extends SubsystemBase {

	WPI_TalonFX rightClimber = new WPI_TalonFX(Constants.RIGHT_CLIMBER);
	WPI_TalonFX leftClimber = new WPI_TalonFX(Constants.LEFT_CLIMBER);

	/** Creates a new Climber. */
	public Climber() {

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}
