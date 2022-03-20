// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftTwoBall extends SequentialCommandGroup {
  /** Creates a new LeftTwoBall. */
  public LeftTwoBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoIntake(),
      new AutoPathSegment("LeftTwoBallForward.wpilib.json"),
      new AutoIntakeStop(),
      new AutoPathSegment("LeftTwoBallBackward.wpilib.json"),
      new AutoShoot(6000, Constants.FEEDER_SHOOTING_SPEED).withTimeout(2.5),
      new AutoShoot(0, 0).withTimeout(0),
      new AutoIntake(),
      new AutoPathSegment("LeftTwoBallDefense.wpilib.json"),
      new TurnRelativeDegrees(90, true),
      new AutoShoot(2000, Constants.FEEDER_SHOOTING_SPEED).withTimeout(2.5),
      new InstantCommand(()->RobotContainer.shooter.stop()),
      new AutoIntakeStop(),
      new InstantCommand(()->RobotContainer.intake.retractIntake())
    );
  }
}
