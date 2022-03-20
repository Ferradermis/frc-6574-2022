// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnRelativeDegrees extends CommandBase {

  double degrees = 0.0;
  boolean clockwise = false;
  double initialGyro = 0.0;

  /** Creates a new TurnRelativeDegrees. */
  public TurnRelativeDegrees(double degrees, boolean clockwise) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.degrees = degrees;
    this.clockwise = clockwise;
    initialGyro = RobotContainer.driveTrain.getGyroAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (clockwise) {
      RobotContainer.driveTrain.arcadeDrive(0.0, -0.5);
    } else {
      RobotContainer.driveTrain.arcadeDrive(0.0, 0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(RobotContainer.driveTrain.getGyroAngle() - initialGyro) < 5) {
      return true;
    } else {
      return false;
    }
  }
}
