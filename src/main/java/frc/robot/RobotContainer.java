/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autocommands.LeftTwoBall;
import frc.robot.commands.autocommands.blinkincommands.Green;
import frc.robot.commands.climbercommands.CloseInitialHook;
import frc.robot.commands.climbercommands.DetachInitialHook;
import frc.robot.commands.climbercommands.DetachSecondHook;
import frc.robot.commands.climbercommands.LowerClimberElevator;
import frc.robot.commands.climbercommands.RaiseClimberElevator;
import frc.robot.commands.climbercommands.ToggleElevator;
import frc.robot.commands.climbercommands.ToggleInitialHook;
import frc.robot.commands.drivetraincommands.ArcadeDrive;
import frc.robot.commands.intakecommands.IntakeProcess;
import frc.robot.commands.shootercommands.HighShooter;
import frc.robot.commands.shootercommands.LowShooter;
import frc.robot.subsystems.Climber;
/*import frc.robot.commands.autonomouscommands.AutonomousMovingPractice;
import frc.robot.commands.drivetraincommands.ArcadeDrive;
import frc.robot.commands.shootercommands.ShootCommand;
import frc.robot.commands.shootercommands.ShootCommandNoLime;
import frc.robot.commands.shootercommands.StopShooting;*/
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
	/**
	* The container for the robot.  Contains subsystems, OI devices, and commands.
	*/

	//Subsystems
	public static final OI oi = new OI(); //Phase out
	public static final DriveTrain driveTrain = new DriveTrain();
	public static final Shooter shooter = new Shooter();
	public static final Intake intake = new Intake();
	public static final Climber climber = new Climber();

	public static final Compressor compressor = new Compressor(Constants.PCH_CAN_ID, PneumaticsModuleType.REVPH);
	public static final PneumaticHub ph = new PneumaticHub(Constants.PCH_CAN_ID);
	public static final PowerDistribution pdh = new PowerDistribution(Constants.PDH_CAN_ID, PowerDistribution.ModuleType.kRev);

	public static final Blinkin m_blinkin = new Blinkin(0);

	//Commands
	public final ArcadeDrive arcadeDrive = new ArcadeDrive();

	public static SendableChooser<CommandBase> autochooser = new SendableChooser<CommandBase>();
	public static SendableChooser<String> allianceChooser = new SendableChooser<String>();
	public static PIDController leftPID = new PIDController(Constants.kPDriveVel, 0, 0);
	public static PIDController rightPID = new PIDController(Constants.kPDriveVel, 0, 0);

	public RobotContainer() {

		driveTrain.setDefaultCommand(arcadeDrive);

		//SmartDashboard.putNumber("Delay Start of Auto: ", 0.0);
		//autochooser.addOption("Auto", new DriveAlongAngle(-1,0));
		SmartDashboard.putData("Autonomous Chooser", autochooser);
		//allianceChooser.setDefaultOption("Red Alliance (pipeline)", "red");
		//allianceChooser.addOption("Blue Alliance (pipeline)", "blue");
		//SmartDashboard.putData("Alliance (pipeline)", allianceChooser);

		configureButtonBindings();
	}

	private void configureButtonBindings() {


		//-----Driver Controls-----\\
		oi.driver_rightBumper.toggleWhenPressed(new IntakeProcess() );// lime
		oi.driver_rightTrigger.whileHeld(new HighShooter());// Hot pink
		oi.driver_leftTrigger.whileHeld(new LowShooter());// blue violet
		oi.driver_aButton.toggleWhenPressed(new ToggleElevator());
		oi.driver_xButton.toggleWhenPressed(new ToggleInitialHook());

		//-----Operator Controls-----\\
		//oi.operator_aButton.toggleWhenPressed(climb, true);  // schedules ClimbUpAndDown for endgame
		oi.operator_leftBumper.whenPressed(new DetachSecondHook());//yellow
		oi.operator_rightBumper.whenPressed(new CloseInitialHook());//red orange
		oi.operator_aButton.whenPressed(new DetachInitialHook());//lawn green
		oi.operator_xButton.whenPressed(new RaiseClimberElevator());//sky blue
		oi.operator_bButton.whenPressed(new LowerClimberElevator());//dark red
		
		oi.operator_yButton.whenPressed(()->climber.incrementClimber()).whenReleased(()->climber.stop());
		//oi.operator_xButton.whenPressed(new SetClimberToStartPosition(Constants.CLIMBER_START_POSITION));
	}


	/**
	* Use this to pass the autonomous command to the main {@link Robot} class.
	*
	* @return the command to run in autonomous
	*/
	public Command getAutonomousCommand() {
		
		 // return autochooser.getSelected();
		  
		 /* var autoVoltageConstraint =
			  new DifferentialDriveVoltageConstraint(
				  new SimpleMotorFeedforward(
					  Constants.ksVolts,
					  Constants.kvVoltSecondsPerMeter,  Constants.kaVoltSecondsSquaredPerMeter),
					  Constants.kDriveKinematics,
					  10);*/
		    // Create config for trajectory
			/*TrajectoryConfig config = 
			new TrajectoryConfig(
					Constants.kMaxSpeedMetersPerSecond,
					Constants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(Constants.kDriveKinematics)
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint);*/
	/*
		// An example trajectory to follow.  All units in meters.
		Trajectory exampleTrajectory =
			TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(4, 0)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(2, 0, new Rotation2d(Math.PI)),
				// Pass config
				config);*/
	
		/*RamseteCommand ramseteCommand =
			new RamseteCommand(
				Robot.autoTrajectory,
				driveTrain::getPose,
				new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
				new SimpleMotorFeedforward(
					Constants.ksVolts,
					Constants.kvVoltSecondsPerMeter,
					Constants.kaVoltSecondsSquaredPerMeter),
				Constants.kDriveKinematics,
				driveTrain::getWheelSpeeds,
				leftPID,
				rightPID,
				// RamseteCommand passes volts to the callback
				driveTrain::tankDriveVolts,
				driveTrain);*/
	
		// Reset odometry to the starting pose of the trajectory.
		//driveTrain.resetOdometry(Robot.autoTrajectories.get(DriverStation.getGameSpecificMessage()).getInitialPose());
		//return Robot.autoPathCommands.get(DriverStation.getGameSpecificMessage()).andThen(() -> driveTrain.tankDriveVolts(0, 0));
		return new LeftTwoBall();	

		/*

		SequentialCommandGroup autoRoutine = new SequentialCommandGroup(
			//shoots low goal(.35)
			new AutoIntake(),

			new AutoDrive(0, -Constants.AUTO_DRIVE_DISTANCE).withTimeout(3.0),
			new AutoDrive(0,0).withTimeout(.1),
			
			new AutoIntakeStop(),

			new AutoDrive(0, Constants.AUTO_DRIVE_DISTANCE).withTimeout(3.0),

			new AutoShoot(Constants.SHOOTER_LOW_GOAL_PERCENT_OUTPUT, Constants.FEEDER_SHOOTING_SPEED).withTimeout(2),
			new WaitCommand(1),
			new AutoShoot(0,0).withTimeout(.25),
			new AutoStopOmnis()
		);

		SequentialCommandGroup simpleAutoRoutine = new SequentialCommandGroup(
			new AutoShoot(Constants.SHOOTER_LOW_GOAL_PERCENT_OUTPUT, Constants.FEEDER_SHOOTING_SPEED).withTimeout(1),
			new WaitCommand(.5),
			new AutoShoot(0,0).withTimeout(.25),
			new AutoDrive(0, Constants.AUTO_DRIVE_DISTANCE).withTimeout(3.0),
			new WaitCommand(1),
			new AutoDrive(0,0).withTimeout(.25)

		);
		
		return autoRoutine;*/
		
	}

	public static String getAlliance() {
		return allianceChooser.getSelected();
	}

}