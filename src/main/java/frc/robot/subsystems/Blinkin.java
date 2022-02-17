// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Blinkin extends SubsystemBase {

	private static Spark blinkin;

	public Blinkin(int pwmPort) {
		blinkin = new Spark(pwmPort);
	}
	public static void set(double val) {
		if ((val >= -1.0) && (val <= 1.0)) {
			blinkin.set(val);
		}
	}

	public static void allianceColor() {
		boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
		if (isRed == true){
			set(-0.01);
			System.out.println("led RED");
		} else {
			set(0.19);
			System.out.println("led BLUE");
		}
	}

	public static void oceanPalette(){
		set(-0.95);
	}
}