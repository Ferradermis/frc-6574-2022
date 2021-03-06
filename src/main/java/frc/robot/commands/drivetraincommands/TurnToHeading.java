/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands.drivetraincommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnToHeading extends CommandBase {
  /**
   * Creates a new TurnToHeading.
   */

  double kF = 0.05;
  double kP = 0.02; 
  double angleError;
  double turnSpeed;
  double tolerance = 3;
  double intendedHeading;
  final double MaxTurnSpeed = 0.25;


  public TurnToHeading(double intendedHeading) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);

    this.intendedHeading = intendedHeading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    //angleError = intendedHeading - driveTrain.getGyroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleError = intendedHeading - RobotContainer.driveTrain.getGyroAngle();   
    turnSpeed = angleError * kP + Math.copySign(kF, angleError);
    // make sure turnSpeed is not greater than MaxTurnSpeed
    turnSpeed = ((Math.abs(turnSpeed) > MaxTurnSpeed ? Math.copySign(MaxTurnSpeed, angleError): turnSpeed));
    RobotContainer.driveTrain.arcadeDrive(0, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(angleError)) <= tolerance);
  }
}
