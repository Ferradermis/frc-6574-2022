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

    /*
        CAN IDs
    */

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
    public static final int INTAKE_LEFT_CAN_ID = 0;
    public static final int INTAKE_RIGHT_CAN_ID = 0;

    public static final int INTAKE_RIGHT_OMNI_CAN_ID = 0;
    public static final int INTAKE_LEFT_OMNI_CAN_ID = 0;

    /** Shooter */
    public static final int SHOOTER_LEFT_CAN_ID = 0;
    public static final int SHOOTER_RIGHT_CAN_ID = 0;

    public static final int FEEDER_INNER_CAN_ID = 0;
    public static final int FEEDER_OUTER_CAN_ID = 0;

    /*
        PCH IDs
    */

    /** Intake */
    public static final int INTAKE_OUT_PCH_ID = 0;
    public static final int INTAKE_IN_PCH_ID = 0;



    /* trajectorie **/
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;

    public static final double kPDriveVel = 0;//3.561;

    public static final double kTrackWidthMeters = 0;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;

    // Explanation:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
    public static final double kRamseteB = 0;
    public static final double kRamseteZeta = 0;

}