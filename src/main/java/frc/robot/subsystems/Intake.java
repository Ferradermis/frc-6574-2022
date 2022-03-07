// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

	WPI_TalonFX intakeLeft = new WPI_TalonFX(Constants.INTAKE_LEFT_CAN_ID);
	WPI_TalonFX intakeRight = new WPI_TalonFX(Constants.INTAKE_RIGHT_CAN_ID);
	CANSparkMax omniLeft = new CANSparkMax(Constants.INTAKE_LEFT_OMNI_CAN_ID, MotorType.kBrushless);
	CANSparkMax omniRight = new CANSparkMax(Constants.INTAKE_RIGHT_OMNI_CAN_ID, MotorType.kBrushless);
	public Solenoid deployer = new Solenoid(Constants.PCH_CAN_ID, PneumaticsModuleType.REVPH, Constants.INTAKE_PCH_ID);

	/** Creates a new Intake. */
	public Intake() {
		intakeLeft.configFactoryDefault();
		intakeRight.configFactoryDefault();
		omniLeft.restoreFactoryDefaults();
		omniRight.restoreFactoryDefaults();

		intakeLeft.follow(intakeRight);
		intakeLeft.setInverted(true);

		intakeLeft.setNeutralMode(NeutralMode.Coast);
		intakeRight.setNeutralMode(NeutralMode.Coast);

		omniLeft.setIdleMode(IdleMode.kCoast);
		omniRight.setIdleMode(IdleMode.kCoast);

		intakeLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 20, 1));
		intakeRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 20, 1));

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//double speed = RobotContainer.oi.getOperatorLeftY();
		/*
		if (Math.abs(speed)< .05) {
		}else {
			hold();
		}*/
		//spin(speed);
		//spinOmnis(speed);
		//RobotContainer.shooter.spinInner(speed);
		//RobotContainer.shooter.spinOuter(speed);
	}

	public void deploy() {
		deployer.set(true);
	}

	public void retract() {
		deployer.set(false);
	}

	public void stop() {
		intakeRight.set(0);
	}

	public void spin(double speed) {
		intakeRight.set(-speed);
	}

	public void stopOmnis() {
		omniRight.set(0);
		omniLeft.set(0);
	}

	public void spinOmnis(double speed) {
		omniLeft.set(speed);
		omniRight.set(-speed);
	}

	public void toggleDeploy() {
		if (deployer.get() == true) {
			retract();
		}
		if (deployer.get() == false) {
			deploy();
		}
	}

}
