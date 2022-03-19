// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

		shooterLeft.setNeutralMode(NeutralMode.Coast);
		shooterRight.setNeutralMode(NeutralMode.Coast);

		feederInner.setNeutralMode(NeutralMode.Brake);
		feederOuter.setNeutralMode(NeutralMode.Brake);

		configShooterPID();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		SmartDashboard.putNumber("Shooter velocity", shooterRight.getSelectedSensorVelocity());
		spinShooterClosedLoop(13000, Constants.FEEDER_SHOOTING_SPEED);
		//spinShooterPercentOutput(0.7, 0);
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

	public void spinShooterClosedLoop(double velocity, double feederPercent) {
		shooterRight.set(ControlMode.Velocity, velocity);
		feederInner.set(ControlMode.PercentOutput, feederPercent);
		feederOuter.set(ControlMode.PercentOutput, feederPercent);
		RobotContainer.intake.spinOmnis(Constants.INTAKE_SPIN_SPEED);

	}

	public void spinShooterPercentOutput(double percent, double feederPercent) {
		shooterRight.set(ControlMode.PercentOutput, percent);
		feederInner.set(ControlMode.PercentOutput, feederPercent);
		feederOuter.set(ControlMode.PercentOutput, feederPercent);
		RobotContainer.intake.spinOmnis(Constants.INTAKE_SPIN_SPEED);

	}

	public void stop() {
		shooterRight.set(0);
		feederInner.set(ControlMode.PercentOutput, 0);
		feederOuter.set(ControlMode.PercentOutput, 0);
		RobotContainer.intake.spinOmnis(0);

	}
	public void configShooterPID() {
		double kP = 0.2;
		double kI = 0;
		double kD = 0;
		double kF = 0.055;
		shooterRight.config_kP(0, kP);
		shooterRight.config_kF(0, kF);
		shooterRight.config_kI(0, kI);
		shooterRight.config_kD(0, kD);
	}
}
