/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

	public static AHRS gyro = new AHRS(I2C.Port.kMXP);
	public WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_CAN_ID);
	public WPI_TalonFX middleLeft = new WPI_TalonFX(Constants.MIDDLE_LEFT_CAN_ID);
	public WPI_TalonFX backLeft = new WPI_TalonFX(Constants.BACK_LEFT_CAN_ID);
	public WPI_TalonFX frontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_CAN_ID);
	public WPI_TalonFX middleRight = new WPI_TalonFX(Constants.MIDDLE_RIGHT_CAN_ID);
	public WPI_TalonFX backRight = new WPI_TalonFX(Constants.BACK_RIGHT_CAN_ID);

	// following variable are used in turnToHeading and driveAlongAngle
	final double MaxDriveSpeed = 0.3;//was .15
	final double MaxTurnSpeed = 0.25;
	public final int EncoderUnitsPerFeet = 14500;//New robot probably need to change.
	public final double encoderDistancePerPulse = 0.000021382 * (0.925);


	private final DifferentialDriveOdometry m_odometry;

	public DriveTrain() {
		configMotors();
		resetPosition();
		gyro.calibrate();//
		m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d().unaryMinus());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//SmartDashboard.putNumber("Actual Gyro Heading: ", gyro.getAngle());
		//SmartDashboard.putNumber("Acual Drive Position: ", getPosition());

		//SmartDashboard.putNumber("Left:", frontLeft.getSelectedSensorPosition() * encoderDistancePerPulse);
		//SmartDashboard.putNumber("Right:", frontRight.getSelectedSensorPosition() * encoderDistancePerPulse);
		
		//SmartDashboard.putNumber("Left speed:", frontLeft.getSelectedSensorVelocity() * encoderDistancePerPulse * 10);
		//SmartDashboard.putNumber("Right speed:", frontRight.getSelectedSensorVelocity() * encoderDistancePerPulse * 10);
		
		//SmartDashboard.putNumber("Left PID:", RobotContainer.leftPID.getSetpoint());
		//SmartDashboard.putNumber("Right PID:", RobotContainer.rightPID.getSetpoint());

		m_odometry.update(gyro.getRotation2d().unaryMinus(), frontLeft.getSelectedSensorPosition() * encoderDistancePerPulse, frontRight.getSelectedSensorPosition() * encoderDistancePerPulse);
	}

	SlewRateLimiter forwardBackLimit = new SlewRateLimiter(2);

	/**
	* Drives the robot using the arcade drive style,
	* @param drive is "speed" to move forward (positive) or backward (negative)
	* @param steer is "amount" to turn right (positive) or left (negative)
	* best to pass in normalized variables from 1 to -1
	*/
	public void arcadeDrive(double drive, double steer) {

		drive = forwardBackLimit.calculate(drive);
		//if steer and drive are both too low, stop the motors and end
		if ((Math.abs(drive) <= 0.05) && (Math.abs(steer) <= 0.05)) {
			stop();
			return;
		}

		double leftSpeed = drive + steer;
		double rightSpeed = drive - steer;

		frontLeft.set(ControlMode.PercentOutput, -leftSpeed);
		frontRight.set(ControlMode.PercentOutput, -rightSpeed);
	}
	public DifferentialDriveWheelSpeeds getWheelSpeeds(){
		return new DifferentialDriveWheelSpeeds(frontLeft.getSelectedSensorVelocity() * encoderDistancePerPulse * 10, frontRight.getSelectedSensorVelocity() * encoderDistancePerPulse * 10);
	  }
	
	  public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	  }
	  public void resetOdometry(Pose2d pose) {
		frontLeft.setSelectedSensorPosition(0);
		frontRight.setSelectedSensorPosition(0);
		m_odometry.resetPosition(pose, gyro.getRotation2d().unaryMinus());
	  }
	
	  public void tankDriveVolts(double leftVolts, double rightVolts) {
		frontLeft.setVoltage(leftVolts);
		frontRight.setVoltage(rightVolts);
	  }

	/**
	* Stops all drivetrain wheels.
	*/
	public void stop() {
		frontLeft.set(ControlMode.PercentOutput, 0);
		frontRight.set(ControlMode.PercentOutput, 0);
	}

	/**
	* Gets the angle of drive train from its initial position.
	* @return	a double containing the drive train's current heading
	*/
	public double getGyroAngle() {
		return -gyro.getAngle();
	}

	/**
	* Resets the drive train's gyroscope position to the zero value.
	*/
	public void resetGyro() {
		gyro.reset();
	}

	/**
	* Gets the current position of the drive train
	* @return	a double containing the drive train's current position;
	*                        as an average of left and right position.
	*/
	public double getPosition() {
		return ((frontLeft.getSelectedSensorPosition() + frontRight.getSelectedSensorPosition()) / 2);
	}

	// NOTE THIS FUNCTION CALL IS NON-BLOCKING; TRY TO AVOID USING
	public void resetPosition() {
		frontLeft.setSelectedSensorPosition(0, 0, 50);
		frontRight.setSelectedSensorPosition(0, 0, 50);
	}

	public void setPosition(int distance) {
		frontLeft.set(ControlMode.Position, distance);
		frontRight.set(ControlMode.Position, distance);
	}

	public void hold() {
		double frontLeftPosition = frontLeft.getSelectedSensorPosition();
		double frontRightPosition = frontRight.getSelectedSensorPosition();
		frontLeft.set(ControlMode.Position, frontLeftPosition);
		frontRight.set(ControlMode.Position, frontRightPosition);
	}

	private void configMotors() {

		double rampRate = 0.1; //time in seconds to go from 0 to full throttle; Lower this number and tune current limits
		int currentLimit = 45;
		//currentLimitThreshold represents the current that the motor needs to sustain for the currentLimitThresholdTime to then be limited to the currentLimit
		int currentLimitThreshold = 60;
		double currentLimitThresholdTime = 1.0;

		gyro.enableLogging(false);

		frontLeft.configFactoryDefault();
		frontRight.configFactoryDefault();
		middleLeft.configFactoryDefault();
		middleRight.configFactoryDefault();
		backLeft.configFactoryDefault();
		backRight.configFactoryDefault();

		//Enables motors to follow commands sent to front and left
		middleRight.follow(frontRight);
		middleLeft.follow(frontLeft);
		backRight.follow(frontRight);
		backLeft.follow(frontLeft);

		frontLeft.configOpenloopRamp(rampRate);
		middleLeft.configOpenloopRamp(rampRate);
		backLeft.configOpenloopRamp(rampRate);
		frontRight.configOpenloopRamp(rampRate);
		middleRight.configOpenloopRamp(rampRate);
		backRight.configOpenloopRamp(rampRate);

		frontRight.setInverted(true);
		middleRight.setInverted(true);
		backRight.setInverted(true);

		frontLeft.setNeutralMode(NeutralMode.Brake);
		middleLeft.setNeutralMode(NeutralMode.Coast);
		backLeft.setNeutralMode(NeutralMode.Coast);
		frontRight.setNeutralMode(NeutralMode.Brake);
		middleRight.setNeutralMode(NeutralMode.Coast);
		backRight.setNeutralMode(NeutralMode.Coast);

		frontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		middleLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		backLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		frontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		middleRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));
		backRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimitThreshold, currentLimitThresholdTime));

		middleLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
		middleLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10000);
		middleLeft.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10000);
		middleLeft.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10000);
		middleLeft.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10000);
		middleLeft.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10000);
		middleLeft.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);

		middleRight.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
		middleRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10000);
		middleRight.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10000);
		middleRight.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10000);
		middleRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10000);
		middleRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10000);
		middleRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);

		backRight.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
		backRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10000);
		backRight.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10000);
		backRight.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10000);
		backRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10000);
		backRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10000);
		backRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);
	
		backLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
		backLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10000);
		backLeft.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10000);
		backLeft.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10000);
		backLeft.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10000);
		backLeft.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10000);
		backLeft.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10000);
	}

}