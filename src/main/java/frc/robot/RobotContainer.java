/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivetraincommands.ArcadeDrive;
/*import frc.robot.commands.autonomouscommands.AutonomousMovingPractice;
import frc.robot.commands.autonomouscommands.MoveOffLine;
import frc.robot.commands.drivetraincommands.ArcadeDrive;
import frc.robot.commands.shootercommands.ShootCommand;
import frc.robot.commands.shootercommands.ShootCommandNoLime;
import frc.robot.commands.shootercommands.StopShooting;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climber;*/
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;


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
	//public static final Shooter shooter = new Shooter();
	public static final Intake intake = new Intake();
	//public static final Climber climber = new Climber();

	public static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

	//public static final Blinkin m_blinkin = new Blinkin(0);

	//Commands
	public final ArcadeDrive arcadeDrive = new ArcadeDrive();

	public static SendableChooser<CommandBase> autochooser = new SendableChooser<CommandBase>();
	public static SendableChooser<String> allianceChooser = new SendableChooser<String>();

	public RobotContainer() {

		driveTrain.setDefaultCommand(arcadeDrive);
		//  turret.setDefaultCommand(turnTurret);

		SmartDashboard.putNumber("Delay Start of Auto: ", 0.0);
		//autochooser.addOption("Move off Initiation line", new MoveOffLine(-1));

		SmartDashboard.putData("Autonomous Chooser", autochooser);
		allianceChooser.setDefaultOption("Red Alliance (pipeline)", "red");
		allianceChooser.addOption("Blue Alliance (pipeline)", "blue");
		SmartDashboard.putData("Alliance (pipeline)", allianceChooser);

		configureButtonBindings();
	}

	private void configureButtonBindings() {

		oi.operator_leftTrigger.whenPressed(()->intake.spin(1))
		.whenReleased(()->intake.stop());
		oi.driver_rightBumper.whenPressed(()->intake.toggleDeploy());
		//-----Driver Controls-----\\
		//oi.driver_rightBumper.whenPressed(()->intake.deployOrRetract());


		//-----Operator Controls-----\\
		//oi.operator_aButton.toggleWhenPressed(climb, true);  // schedules ClimbUpAndDown for endgame
		//oi.operator_rightTrigger.whenPressed(new ShootCommand()).whenReleased(new StopShooting());
	}

	/**
	* Use this to pass the autonomous command to the main {@link Robot} class.
	*
	* @return the command to run in autonomous
	*/
	public Command getAutonomousCommand() {
		return autochooser.getSelected();
	}

	public String getAlliance() {
		return allianceChooser.getSelected();
	}

}