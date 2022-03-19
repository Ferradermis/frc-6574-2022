// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**Open Loop Constants */
    public static final double INTAKE_SPIN_SPEED = .8;  
    public static final double OMNIS_SPIN_SPEED = 0.75;
    public static final double FEEDER_INNER_SPEED = 0.7;
    public static final double FEEDER_OUTER_SPEED = 0.5;
    public static final double FEEDER_SHOOTING_SPEED = 0.75;
    public static final double SHOOTER_LOW_GOAL_PERCENT_OUTPUT = 0.35;
    public static final double SHOOTER_HIGH_GOAL_PERCENT_OUTPUT = 0.55;
    public static final double AUTO_DRIVE_DISTANCE = 0.25;

    /**Closed Loop Constants */
    public static final double SHOOTER_RESTING_VELOCITY = 10; //10 is placeholder value
    public static final double SHOOTER_VELOCITY_ONE = 20; //20 is placeholder value
    public static final double INTAKE_VELOCITY = 30;

    /*
        CAN IDs
    */

    public static final int PCH_CAN_ID = 9;
    public static final int PDH_CAN_ID = 10;

    /** DriveTrain */
    public static final int FRONT_LEFT_CAN_ID = 1;
    public static final int MIDDLE_LEFT_CAN_ID = 2;
    public static final int BACK_LEFT_CAN_ID = 3;
    public static final int FRONT_RIGHT_CAN_ID = 4;
    public static final int MIDDLE_RIGHT_CAN_ID = 5;
    public static final int BACK_RIGHT_CAN_ID = 6;

    //public static AHRS gyro = new AHRS(I2C.Port.kMXP);

    /** Climber */
    public static final int CLIMBER_RIGHT_CAN_ID = 8;
    public static final int CLIMBER_LEFT_CAN_ID = 7;

    /** Intake */
    public static final int INTAKE_LEFT_CAN_ID = 17;
    public static final int INTAKE_RIGHT_CAN_ID = 16;

    public static final int INTAKE_RIGHT_OMNI_CAN_ID = 12;
    public static final int INTAKE_LEFT_OMNI_CAN_ID = 11;

    /** Shooter */
    public static final int SHOOTER_LEFT_CAN_ID = 15;
    public static final int SHOOTER_RIGHT_CAN_ID = 13;

    public static final int FEEDER_INNER_CAN_ID = 14;
    public static final int FEEDER_OUTER_CAN_ID = 18;

    /*
        PCH IDs
    */

    /** Climber */
    public static final int CLIMBER_INITIAL_HOOK_PCH_ID = 6;
    public static final int CLIMBER_SECOND_HOOK_PCH_ID = 7;

    /** Intake */
    public static final int INTAKE_PCH_ID = 15;

    /* trajectory **/
    public static final double ksVolts = 0.69156;
    public static final double kvVoltSecondsPerMeter = 2.5347;
    public static final double kaVoltSecondsSquaredPerMeter = 0.30969;

    public static final double kPDriveVel = 3.4345;//kPDriveVel = 3.193;//3.561;

    public static final double kTrackWidthMeters = 0.635;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    //public static final double kMaxSpeedMetersPerSecond = 1.5;
    //public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    // Explanation:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;


    public double convertTalonFXTicksToRPM(double TalonFXTicks) {
        double convertedTalonFXTicks = TalonFXTicks;
    
        return convertedTalonFXTicks;
    }

}