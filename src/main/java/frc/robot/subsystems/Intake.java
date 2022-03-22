// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

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

		intakeLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
		intakeLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
		intakeLeft.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
		intakeLeft.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);

		intakeRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
		intakeRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
		intakeRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);

		omniLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		omniLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);

		omniRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		omniRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);





	}

	@Override
	public void periodic() {

	}

	public void deployIntake() {
		deployer.set(true);
	}

	public void retractIntake() {
		deployer.set(false);
	}

	public void stop() {
		intakeRight.set(0);
	}

	public void spin(double speed) {
		intakeRight.set(-speed);
	}

	public void spinClosedVelocity(double speed) {
		intakeRight.set(ControlMode.Velocity, speed);
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
			retractIntake();
		}
		if (deployer.get() == false) {
			deployIntake();
		}
	}

	public void configPID() {
		double kF = 0;
		double kP = 0;
		
		intakeRight.config_kF(0, kF);
		intakeRight.config_kP(0, kP);

	}

}
