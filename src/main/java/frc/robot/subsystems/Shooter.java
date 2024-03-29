// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

	public WPI_TalonFX feederInner = new WPI_TalonFX(Constants.FEEDER_INNER_CAN_ID);
	public WPI_TalonFX feederOuter = new WPI_TalonFX(Constants.FEEDER_OUTER_CAN_ID);
	public WPI_TalonFX shooterLeft = new WPI_TalonFX(Constants.SHOOTER_LEFT_CAN_ID);
	public WPI_TalonFX shooterRight = new WPI_TalonFX(Constants.SHOOTER_RIGHT_CAN_ID);

	CANSparkMax topRoller = new CANSparkMax(Constants.SHOOTER_TOPROLLER_CAN_ID, MotorType.kBrushless);
	SparkMaxPIDController topRollerPIDController = topRoller.getPIDController();
	RelativeEncoder topRollerEncoder = topRoller.getEncoder();


	/** Creates a new Shooter. */
	public Shooter() {
		configMotors();
		configShooterPID();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//SmartDashboard.putNumber("Shooter velocity", shooterRight.getSelectedSensorVelocity());
		//SmartDashboard.putNumber("Top roller velocity", topRollerEncoder.getVelocity());

		//spinShooterClosedLoop(13000, Constants.FEEDER_SHOOTING_SPEED);
		//spinShooterPercentOutput(0.7, 0);
	}

	public void stopShooterWheels() {
		shooterRight.set(0);
	}

	public void restingShooterSpeed() {
		shooterRight.set(Constants.SHOOTER_RESTING_VELOCITY);
	}

	public void spinInner(double speed) {
		feederInner.set(speed);
	}

	public void stopInner() {
		feederInner.set(0);
	}

	public void spinOuter(double speed) {
		feederOuter.set(speed);
	}

	public void stopOuter() {
		feederOuter.set(0);
	}

	public void spinTopRollerClosedLoop (double velocity) {
		//topRollerPIDController.setReference(velocity, ControlType.kVelocity);
	}

	public void spinTopRollerOpenLoop(double speed) {
		topRoller.set(speed);
	}

	public void stopTopRoller() {
		topRoller.set(0);
	}

	public void spinShooterClosedLoop(double velocity, double feederPercent) {
		shooterRight.set(ControlMode.Velocity, velocity);
		shooterLeft.set(ControlMode.Velocity, velocity);
		feederInner.set(ControlMode.PercentOutput, feederPercent);
		feederOuter.set(ControlMode.PercentOutput, feederPercent);
		RobotContainer.intake.spinOmnis(Constants.INTAKE_SPIN_SPEED);
		spinTopRollerOpenLoop(Constants.TOPROLLER_OPEN_LOOP);
	}

	public void spinShooterClosedLoopLower(double velocity, double feederPercent) {
		shooterRight.set(.35);
		feederInner.set(ControlMode.PercentOutput, feederPercent);
		feederOuter.set(ControlMode.PercentOutput, feederPercent);
		RobotContainer.intake.spinOmnis(Constants.INTAKE_SPIN_SPEED);
		spinTopRollerOpenLoop(Constants.TOPROLLER_OPEN_LOOP_LOWER_GOAL);
	}

	public void spinShooterPercentOutput(double percent, double feederPercent) {
		shooterRight.set(ControlMode.PercentOutput, percent);
		shooterLeft.set(ControlMode.PercentOutput, percent);
		feederInner.set(ControlMode.PercentOutput, feederPercent);
		feederOuter.set(ControlMode.PercentOutput, feederPercent);
		RobotContainer.intake.spinOmnis(Constants.INTAKE_SPIN_SPEED);
	}

	public void stop() {
		shooterRight.set(0); //Constants.SHOOTER_RESTING_VELOCITY
		shooterLeft.set(0);
		feederInner.set(ControlMode.PercentOutput, 0);
		feederOuter.set(ControlMode.PercentOutput, 0);
		RobotContainer.intake.spinOmnis(0);
		stopTopRoller();
	}

	public void configShooterPID() {
		//Main Wheels
		double kP = 0.2;
		double kI = 0;
		double kD = 0;
		double kF = 0.055;
		shooterRight.config_kP(0, kP);
		shooterRight.config_kF(0, kF);
		shooterRight.config_kI(0, kI);
		shooterRight.config_kD(0, kD);

		shooterLeft.config_kP(0, kP);
		shooterLeft.config_kF(0, kF);
		shooterLeft.config_kI(0, kI);
		shooterLeft.config_kD(0, kD);

		//Top Roller
		double REVkP = .1;
		double REVkF = .0;
		topRollerPIDController.setP(REVkP);
		topRollerPIDController.setFF(REVkF);
	}

	public void configMotors() {
		topRoller.restoreFactoryDefaults();
		topRoller.setIdleMode(IdleMode.kCoast);
		topRoller.setInverted(true);
		topRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
		topRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5000);
		topRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 5000);

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

		shooterLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
		shooterLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10000);
		shooterLeft.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10000);
		shooterLeft.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10000);
		shooterLeft.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10000);
		shooterLeft.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10000);
		shooterLeft.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);

		shooterRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10000);
		shooterRight.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10000);
		shooterRight.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10000);
		shooterRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10000);
		shooterRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10000);
		shooterRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);


	}
}
