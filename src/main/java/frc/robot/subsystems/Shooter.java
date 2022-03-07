// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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
		shooterLeft.setInverted(true);

		feederInner.setInverted(false);
		feederOuter.setInverted(false);

		feederInner.setNeutralMode(NeutralMode.Brake);
		feederOuter.setNeutralMode(NeutralMode.Brake);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void spinInner(double speed) {
		feederInner.set(speed);
	}

	public void spinOuter(double speed) {
		feederOuter.set(speed);
	}

	public void stopInner() {
		feederInner.set(0);
	}

	public void stopOuter() {
		feederOuter.set(0);
	}

	public void spinShooter(double velocity) {
		shooterRight.set(ControlMode.Velocity, velocity);
	}

	public void spinShooterPercentOutput(double percent, double feederPercent) {
		shooterRight.set(ControlMode.PercentOutput, percent);
		feederInner.set(ControlMode.PercentOutput, feederPercent);
		feederOuter.set(ControlMode.PercentOutput, feederPercent);
		RobotContainer.intake.spinOmnis(.35);

	}

	public void stop() {
		shooterRight.set(0);
		feederInner.set(0);
	}
	public void configShooterPID() {
		//double kF = 0.00070;
		//double kP = 0.0032;
		//double kI = 0;
		//double kD = 0;
		//shooterLeft.config_kP(0, kP);
		//shooterRight.config_kP(0, kP);
		//shooterLeft.config_kF(0, kF);
		//shooterRight.config_kF(0, kF);
	}
}
