// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
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
	
	public WPI_TalonFX climberRight = new WPI_TalonFX(Constants.CLIMBER_RIGHT_CAN_ID);
	public WPI_TalonFX climberLeft = new WPI_TalonFX(Constants.CLIMBER_LEFT_CAN_ID);

	double currentLimit = 65; //amps
	double currentLimitThreshold = 65; //amps
	double currentLimitThresholdTime = .5; //seconds

	public Solenoid initialHook = new Solenoid(Constants.PCH_CAN_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_INITIAL_HOOK_PCH_ID);
	public Solenoid secondHook = new Solenoid(Constants.PCH_CAN_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_SECOND_HOOK_PCH_ID);
	public Solenoid elevator = new Solenoid(Constants.PCH_CAN_ID, PneumaticsModuleType.REVPH, Constants.CLIMBER_ELEVATOR_PCH_ID);


	/** Creates a new Climber. */
	public Climber() {
		
		resetClimberEncoder();
		configPID();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//SmartDashboard.putNumber("Climber Position", climberRight.getSelectedSensorPosition());
		//SmartDashboard.putBoolean("Climber at requested position?", climberAtPosition(Constants.CLIMBER_START_POSITION));
		//SmartDashboard.putNumber("Climber Current", climberRight.getSupplyCurrent());

		
		spin(.85 * RobotContainer.oi.getOperatorLeftY());
		}
	

	//if (Math.abs(speed)< .25) {
		//hold();
	//}
	//else {
	public void spin(double speed) {
		climberRight.set(-speed); 
	}

	public void hold() {
		double climberPosition = climberRight.getSelectedSensorPosition();
		climberRight.set(ControlMode.Position, climberPosition);
	}

	public void setToVerticalStart(double verticalClimbPosition) {
		double climbPosition;
		climbPosition = verticalClimbPosition;
		climberRight.set(ControlMode.Position, climbPosition);
	}

	public void incrementClimber() {
		double currentPosition = climberRight.getSelectedSensorPosition();
		climberRight.set(ControlMode.Position, currentPosition+1000);
	}

	public void raiseClimberElevator() {
		elevator.set(true);
	}

	public void lowerClimberElevator() {
		elevator.set(false);
	}

	public void raiseInitialHook() {
		initialHook.set(true);
	}

	public void lowerInitialHook(){ 
		initialHook.set(false);
	}

	public void detachSecondHook() {
		secondHook.set(true);
	}

	public void lowerSecondHook(){ 
		secondHook.set(false);
	}

	public void stop() {
		climberRight.set(0);
	}

	public boolean climberAtPosition(double targetClimberPosition) {
		double tolerance = 300;
		return (Math.abs(climberRight.getSelectedSensorPosition() - targetClimberPosition) < tolerance);
	}

	public void resetClimberEncoder() {
		climberRight.setSelectedSensorPosition(0);
	}

	public void configPID() {
		double kP = 0.1;
		double kI = 0;
		double kD = 0;
		double kF = 0;
		climberRight.config_kP(0, kP);
		climberRight.config_kF(0, kF);
		climberRight.config_kI(0, kI);
		climberRight.config_kD(0, kD);
	}

	public void configMotors() {
		climberRight.configFactoryDefault();
		climberLeft.configFactoryDefault();

		climberLeft.follow(climberRight);
		climberLeft.setInverted(true);
		climberRight.setNeutralMode(NeutralMode.Brake);
		climberLeft.setNeutralMode(NeutralMode.Brake);

		/**CTRE documentation says SupplyCurrentLimit is for avoiding the tripping of breakers*/
		climberLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		climberRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		/**CTRE documentation says StatorCurrentLimit is for limiting acceleration/torque or heat generation*/
		//climberLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		//climberRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));


		climberLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
		climberLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10000);
		climberLeft.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10000);
		climberLeft.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10000);
		climberLeft.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10000);
		climberLeft.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10000);
		climberLeft.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);

		climberRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10000);
		climberRight.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10000);
		climberRight.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10000);
		climberRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10000);
		climberRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10000);
		climberRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);
	}
}
