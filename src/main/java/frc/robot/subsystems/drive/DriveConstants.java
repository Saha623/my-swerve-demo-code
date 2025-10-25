// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final int FRONTLEFT_DRIVEMOTOR_CANID = 1; // This is how we set up driving motors
  public static final int FRONTRIGHT_DRIVEMOTOR_CANID = 2;
  public static final int BACKLEFT_DRIVEMOTOR_CANID = 3;
  public static final int BACKRIGHT_DRIVEMOTOR_CANID = 4;

  public static final int FRONTLEFT_TURNMOTOR_CANID =
      5; // This us how we set up turning motors, its the same as the driving motors above
  public static final int FRONTRIGHT_TURNMOTOR_CANID = 6;
  public static final int BACKLEFT_TURNMOTOR_CANID = 7;
  public static final int BACKRIGHT_TURNMOTOR_CANID = 8;

  public static final int PIGEON_CAN_ID = 9;

  public static final double maxSpeed =
      1.0; // meters per second, max # of meters it can move in a second
  public static final double maxAccel = 1.0; // max acceleration, rate it can speed up
  public static final double odometryFrequency = 1.0; // hertz, rate in which we update odometry
  public static final double trackWidth =
      Units.inchesToMeters(27); // width of the track, or the drive-base
  public static final double trackLength =
      Units.inchesToMeters(27); // length of the track, or the drive-base
  public static final double driveBaseRadius =
      Math.hypot(trackWidth / 2.0, trackLength / 2.0); // radius of the drive-base
  protected static final Translation2d[] moduleTranslations =
      new Translation2d[] { // for locating the robot on the field
        new Translation2d(trackLength / 2.0, trackWidth / 2.0), // front left
        new Translation2d(trackLength / 2.0, -trackWidth / 2.0), // front right
        new Translation2d(-trackLength / 2.0, trackWidth / 2.0), // back left
        new Translation2d(-trackLength / 2.0, -trackWidth / 2.0) // back right
      };

  public static final Rotation2d FrontLeftZeroRotation =
      Rotation2d.fromDegrees(0.0); // zeroes the rotation of the motors
  public static final Rotation2d FrontRightZeroRotation = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d BackLeftZeroRotation = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d BackRightZeroRotation = Rotation2d.fromDegrees(0.0);

  public static final int driveCurrentLimit = 30; // current limit for driving motors
  public static final double wheelRadius = Units.inchesToMeters(1.5);
  public static final double driveReduction =
      (45.0 * 22.0) / (14.0 * 15.0); // 14 pinion teeth and 22 spur teeth for MAXSwerve motors
  public static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);

  public static final double drivePositionFactor =
      2 * Math.PI / driveReduction; // Rotor rotations -> wheel radians
  public static final double driveVelocityFactor =
      2 * Math.PI / 60.0 / driveReduction; // Rotor RPM -> Wheel radians/second

  public static final double DRIVE_KP = 0.0; // PID configurations
  public static final double DRIVE_KD = 0.0;
  public static final double DRIVE_KS = 0.0;
  public static final double DRIVE_KV = 0.0;

  public static final boolean turnInverted = false; // if the turning motors are inverted
  public static final int turnCurrentLimit = 20; // current limit for turning motors
  public static final double turnReduction = 9424.0 / 203.0;
  public static final DCMotor TURN_GEARBOX = DCMotor.getNEO(1);

  public static final boolean turnEncoderInverted = true; // if the turning encoders are inverted
  public static final double turnPositionFactor = 2 * Math.PI; // Rotor rotations -> wheel radians
  public static final double turnVelocityFactor =
      2 * (Math.PI) / 60.0; // Rotor RPM -> Wheel radians/second

  public static final double TURN_KP = 1.9;
  public static final double TURN_KD = 0.0;
  public static final double turnMinInput = 0.0; // Both of the Max and Min inputs are in radians
  public static final double turnMaxInput = 2 * Math.PI;
}
