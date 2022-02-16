// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  WPI_TalonFX intake1 = new WPI_TalonFX(Constants.INTAKE_ONE_CAN_ID);
  WPI_TalonFX intake2 = new WPI_TalonFX(Constants.INTAKE_TWO_CAN_ID);
  DoubleSolenoid deployer = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_OUT_PCH_ID, Constants.INTAKE_IN_PCH_ID);
  CANSparkMax leftOmni = new CANSparkMax(Constants.INTAKE_LEFT_OMNI_CAN_ID, MotorType.kBrushless);
  CANSparkMax rightOmni = new CANSparkMax(Constants.INTAKE_RIGHT_OMNI_CAN_ID, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {
    intake1.configFactoryDefault();
    intake2.configFactoryDefault();
    intake2.follow(intake1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deploy() {
    deployer.set(Value.kForward);
  }

  public void retract(){
    deployer.set(Value.kReverse);
  }

  public void stop(){
    intake1.set(ControlMode.PercentOutput, 0);
  }

  public void spin(double speed) {
    if (deployer.get() == Value.kForward) {
      intake1.set(ControlMode.PercentOutput, speed);
    }
  }
  public void toggleDeploy(){
    if (deployer.get() == Value.kForward) {
      retract();
    }
    if (deployer.get() == Value.kReverse){
      deploy();
    }
  }

}
