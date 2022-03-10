// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


//Single hook
//Double hook, firstHook, secondHook
public class Climber extends SubsystemBase {

	WPI_TalonFX climberRight = new WPI_TalonFX(Constants.CLIMBER_RIGHT_CAN_ID);
	WPI_TalonFX climberLeft = new WPI_TalonFX(Constants.CLIMBER_LEFT_CAN_ID);

	double currentLimit = 60; //amps
	double currentLimitThreshold = 80; //amps
	double currentLimitThresholdTime = .5; //seconds

	public Solenoid initialHook = new Solenoid(Constants.PCH_CAN_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_INITIAL_HOOK_PCH_ID);
	public Solenoid secondHook = new Solenoid(Constants.PCH_CAN_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SECOND_HOOK_PCH_ID);

	/** Creates a new Climber. */
	public Climber() {
		climberRight.configFactoryDefault();
		climberLeft.configFactoryDefault();
		climberLeft.follow(climberRight);
		climberLeft.setInverted(true);
		climberRight.setNeutralMode(NeutralMode.Brake);
		climberLeft.setNeutralMode(NeutralMode.Brake);

		//CTRE documentation says SupplyCurrentLimit is for avoiding the tripping of breakers
		climberLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		climberRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));

		//CTRE documentation says StatorCurrentLimit is for limiting acceleration/torque or heat generation
		//climberLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		//climberRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		double speed = RobotContainer.oi.getOperatorLeftY();
		
		if (Math.abs(speed)< .3) {
			hold();

		}
		else {
			spin(speed);
		}
		
		
	}

	public void spin(double speed) {
		climberRight.set(-speed); 
	}

	public void hold() {
		double climberPosition = climberRight.getSelectedSensorPosition();
		climberRight.set(ControlMode.Position, climberPosition);
	}

	public void stop() {
		climberRight.set(0);
	}

}
