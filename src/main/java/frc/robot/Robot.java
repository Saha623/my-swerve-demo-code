// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private Command autonomousCommand;
  private final RobotContainer robotContainer;
  private final Field2d autofield = new Field2d();
  private final Field2d telefield = new Field2d();
  private Timer timer = new Timer();
  private String autoName;

  public Robot() {

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("isRedAlliance", isRedAlliance());
    
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  @SuppressWarnings("CallToPrintStackTrace")
  public void disabledPeriodic() {
    if (timer.hasElapsed(Constants.RECHECKSECONDS)) {
      updateAutoDisplay(autoName);
      timer.restart();
    }

    String newAutoName;
    newAutoName = robotContainer.getAutonomousCommand().getName();
    if (autoName == null ? (newAutoName != null) : !autoName.equals(newAutoName)) {
        autoName = newAutoName;
        updateAutoDisplay(autoName);
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autofield.setRobotPose(robotContainer.getPose());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      Shuffleboard.selectTab("Teleoperated");
    }

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    telefield.setRobotPose(robotContainer.getPose());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance()
            .filter(value -> value == DriverStation.Alliance.Red)
            .isPresent();
  }

  @SuppressWarnings("CallToPrintStackTrace")
  public void updateAutoDisplay(String name) {
      if (AutoBuilder.getAllAutoNames().contains(name)) {
          System.out.println("Displaying " + name);
          List<PathPlannerPath> pathPlannerPaths;
          try {
              pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(name);

              List<Pose2d> poses = new ArrayList<>();
              for (PathPlannerPath path : pathPlannerPaths) {
                poses.addAll(path.getAllPathPoints().stream()
                        .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                        .collect(Collectors.toList()));
              }
              autofield.getObject("path").setPoses(poses);
              robotContainer.resetPose(poses.get(0));

          } catch (IOException | org.json.simple.parser.ParseException e) {
              DriverStation.reportError(e.getMessage(), false);
          }
      }
  }
}
