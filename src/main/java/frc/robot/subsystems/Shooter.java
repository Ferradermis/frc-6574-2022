// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

	WPI_TalonFX feederInner = new WPI_TalonFX(Constants.FEEDER_INNER_CAN_ID);
	WPI_TalonFX feederOuter = new WPI_TalonFX(Constants.FEEDER_OUTER_CAN_ID);
	WPI_TalonFX shooterLeft = new WPI_TalonFX(Constants.SHOOTER_LEFT_CAN_ID);
	WPI_TalonFX shooterRight = new WPI_TalonFX(Constants.SHOOTER_RIGHT_CAN_ID);

	/** Creates a new Shooter. */
	public Shooter() {
		feederInner.configFactoryDefault();
		feederOuter.configFactoryDefault();
		shooterLeft.configFactoryDefault();
		shooterRight.configFactoryDefault();
		shooterLeft.follow(shooterRight);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void spinInnner(double speed){
		feederInner.set(speed);
	}

	public void spinOuter(double speed){
		feederOuter.set(speed);
	}

	public void stopInner(){
		feederInner.set(0);
	}

	public void stopOuter(){
		feederOuter.set(0);
	}

	public void spinShooter(double velocity){
		shooterRight.set(ControlMode.Velocity, velocity);
	}

	public void stopShooter(){
		shooterRight.set(0);
	}

}
