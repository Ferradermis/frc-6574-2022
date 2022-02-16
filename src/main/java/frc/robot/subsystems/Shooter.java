// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonFX innerFeeder = new WPI_TalonFX(Constants.INNER_FEEDER);
  WPI_TalonFX outerFeeder = new WPI_TalonFX(Constants.OUTER_FEEDER);
  /** Creates a new Shooter. */  
  public Shooter() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
