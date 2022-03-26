// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/ 
public class Robot extends TimedRobot {

	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;
	public static Trajectory autoTrajectory = new Trajectory();
	public static HashMap<String, Trajectory> autoTrajectories = new HashMap<String, Trajectory>();
	public static HashMap<String, RamseteCommand> autoPathCommands = new HashMap<String, RamseteCommand>();

	/**
	* This function is run when the robot is first started up and should be used for any
	* initialization code.
	*/
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();
		try {
			File[] fileList = Filesystem.getDeployDirectory().toPath().resolve("output/").toFile().listFiles();
			for (File file : fileList) {
				if (file.getName().endsWith(".json")) {
					Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(file.toPath());
					RamseteCommand ramseteCommand =
					new RamseteCommand(
						trajectory,
						RobotContainer.driveTrain::getPose,
						new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
						new SimpleMotorFeedforward(
							Constants.ksVolts,
							Constants.kvVoltSecondsPerMeter,
							Constants.kaVoltSecondsSquaredPerMeter),
						Constants.kDriveKinematics,
						RobotContainer.driveTrain::getWheelSpeeds,
						RobotContainer.leftPID,
						RobotContainer.rightPID,
						// RamseteCommand passes volts to the callback
						RobotContainer.driveTrain::tankDriveVolts,
						RobotContainer.driveTrain);
					autoTrajectories.put(file.getName(), trajectory);
					autoPathCommands.put(file.getName(), ramseteCommand);
					//DriverStation.reportWarning(file.getName(), true);
					//System.out.println(file.getName());
				}
			}
			//Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/AutoPath.wpilib.json");
			//autoTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		  } catch (IOException ex) {
			  //DriverStation.reportError("Unable to open trajectory: " + "paths/AutoPath.wpilib.json", ex.getStackTrace());
		  }
		RobotContainer.ph.clearStickyFaults();
		RobotContainer.pdh.clearStickyFaults();
	}

	/**
	* This function is called every robot packet, no matter the mode. Use this for items like
	* diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	*
	* <p>This runs after the mode specific periodic functions, but before LiveWindow and
	* SmartDashboard integrated updating.
	*/
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {

	}

	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		RobotContainer.ph.clearStickyFaults();
		RobotContainer.pdh.clearStickyFaults();
		RobotContainer.driveTrain.resetPosition();
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		if (OI.driver_startButton.get()) { //disables shooter and compressor for endgame
			//RobotContainer.compressor.setClosedLoopControl(false);
			//RobotContainer.compressor.disable();
			RobotContainer.shooter.stopShooterWheels();
		  }
		  else if (OI.driver_backButton.get()) { //enables shooter and compressor for standard teleop
			//RobotContainer.compressor.setClosedLoopControl(true);
			RobotContainer.compressor.enableDigital();
			RobotContainer.shooter.restingShooterSpeed();
		}
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
		RobotContainer.ph.clearStickyFaults();
		RobotContainer.pdh.clearStickyFaults();
		RobotContainer.compressor.enableDigital();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
		//Blinkin.oceanPalette();
		RobotContainer.compressor.enableDigital();
	}

	@Override
	public void testExit() {
		RobotContainer.climber.climberLeft.follow(RobotContainer.climber.climberRight);
		
		RobotContainer.driveTrain.middleRight.follow(RobotContainer.driveTrain.frontRight);
		RobotContainer.driveTrain.middleLeft.follow(RobotContainer.driveTrain.frontRight);
		RobotContainer.driveTrain.backRight.follow(RobotContainer.driveTrain.frontRight);
		RobotContainer.driveTrain.backLeft.follow(RobotContainer.driveTrain.frontRight);
		
		RobotContainer.intake.intakeLeft.follow(RobotContainer.intake.intakeRight);

		RobotContainer.shooter.shooterLeft.follow(RobotContainer.shooter.shooterRight);
	}

}
