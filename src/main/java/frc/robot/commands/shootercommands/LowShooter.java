// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Blinkin;

public class LowShooter extends CommandBase {
  /** Creates a new LowShooter. */
  public LowShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.spinShooterClosedLoopLower(Constants.SHOOTER_LOW_GOAL_PERCENT_OUTPUT, Constants.FEEDER_SHOOTING_SPEED);
    Blinkin.violet();
    //Blinkin.blueViolet();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stop();
    RobotContainer.intake.stopOmnis();
    RobotContainer.shooter.stopInner();
    RobotContainer.shooter.stopOuter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
