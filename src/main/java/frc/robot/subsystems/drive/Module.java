// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.wheelRadius;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  /** Creates a new Module. */
  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError); // Sends an error message if a drive motor disconnects
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".",
            AlertType.kError); // Sends an error message if a turning motor disconnects
  }

  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    int sampleCount = inputs.odometryTimestamps.length; // Calculates the odometry positions
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositionsRad[i]
              * wheelRadius; // Gets the position of the robot every time it updates
      Rotation2d angle =
              inputs.odometryTurnPositions[i]; // Gets the angle of the robot every time it updates
      odometryPositions[i] =
          new SwerveModulePosition(
              positionMeters,
              angle); // Combines both factors to get position and angle of the robot in this update

      driveDisconnectedAlert.set(!inputs.driveConnected); // Updates the error messages and alerts
      turnDisconnectedAlert.set(!inputs.turnConnected);
    }
  }

  public void runSetPoint(
      SwerveModuleState state) { // Runs this to mutate the current state and optimize it
    state.optimize(getAngle()); // Optimizes the current velocity setpoint
    state.cosineScale(inputs.turnPosition);

    io.setDriveVelocity(
        state.speedMetersPerSecond / wheelRadius); // Sets the position of the driving motor
    io.setTurnPosition(state.angle); // Sets the position of the turning motor
  }

  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadius;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadius;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  public double getFFChaaracterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
