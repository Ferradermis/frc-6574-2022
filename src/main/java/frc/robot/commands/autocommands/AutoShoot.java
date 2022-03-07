// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */

  double shooter_speed;
  double feeder_speed;
  public AutoShoot(double shooter, double feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter_speed = shooter;
    feeder_speed = feeder;
    addRequirements(RobotContainer.shooter, RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.spinShooterPercentOutput(shooter_speed, feeder_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
