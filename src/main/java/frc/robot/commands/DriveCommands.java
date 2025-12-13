// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommands {
  private static final double deadband = 0.1;
  private static final double angleKP = 5.0;
  private static final double angleKD = 0.4;
  private static final double angleMaxVelocity = 8.0;
  private static final double angleMaxAcceleration = 20.0;
  private static final double ffStartDelay = 2.0; //seconds
  private static final double ffRampRate = 0.1; //volts per second
  private static final double wheelRadiusMaxVelocity = 0.25; //radians per second
  private static final double wheelRadiusRampRate = 0.05; //radian per second
  private static final Alert alert = new Alert("Drive Characterization", AlertType.kInfo);

  /** Creates a new DriveCommands. */
  public DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
  }

  public static Command joystickDrive(
    Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
          () -> {
            Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadband);

            omega = Math.copySign(omega * omega, omega);

            ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());

              boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
              speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
              drive.runVelocity(speeds);
          },
          drive);
    }

  public static Command joystckDriveAtAngle(
    Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {
      ProfiledPIDController angleController = new ProfiledPIDController(angleKP, 0.0, angleKD, new TrapezoidProfile.Constraints(angleMaxVelocity, angleMaxAcceleration));
      angleController.enableContinuousInput(-Math.PI, Math.PI);

      return Commands.run(
        () -> {
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = angleController.calculate(drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

          ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(), 
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), 
            omega);

          boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
          drive.runVelocity(speeds);
        },
        drive)

        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }
  
  public static Command FeedForwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
      Commands.runOnce(() -> {
        velocitySamples.clear();
        voltageSamples.clear();
  }),

      Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(ffStartDelay),

      Commands.runOnce(timer::restart),

      Commands.run(
        () -> {
          double voltage = timer.get() * ffRampRate;
          drive.runCharacterization(voltage);
          velocitySamples.add(drive.getFFChaaracterizationVelocity());
          voltageSamples.add(voltage);
      },
      drive)

      .finallyDo(() -> {
        int n = velocitySamples.size();
        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumX2 = 0.0;
        for (int i = 0; i < n; i++) {
          sumX += velocitySamples.get(i);
          sumY += voltageSamples.get(i);
          sumXY += velocitySamples.get(i) * voltageSamples.get(i);
          sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
        }
        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

        NumberFormat formatter = new DecimalFormat("#0.00000");

        System.out.println("******* Drive FF Characterization Results *******");
        System.out.println("\tkS: " + formatter.format(kS));
        System.out.println("\tkV: " + formatter.format(kV));
      }));
  }

  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
      Commands.sequence(
        Commands.runOnce(() -> limiter.reset(0.0)),

        Commands.run(
          () -> {
            double speed = limiter.calculate(wheelRadiusMaxVelocity);
            drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
          },
          drive)),

      Commands.sequence(
        Commands.waitSeconds(1.0),

        Commands.runOnce(() -> {
          state.positions = drive.getWheelRadiusCharacterizationPosition();
          state.lastAngle = drive.getRotation();
          state.gyroDelta = 0.0;
        }),

        Commands.run(() -> {
          var rotation = drive.getRotation();
          state.gyroDelta += Math.abs(
            rotation.minus(state.lastAngle).getRadians());
            state.lastAngle = rotation;
        })

        .finallyDo(() -> {
          double[] positions = drive.getWheelRadiusCharacterizationPosition();
          double wheelDelta = 0.0;
          for (int i = 0; i < 4; i++) {
            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
          }
          double wheelRadius = (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

          NumberFormat formatter = new DecimalFormat("#0.000");
          System.out.println("******* Wheel Radius Characterization Results *******");
          System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
          System.out.println(
            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
          System.out.println("\tWheel Radius: "
          + formatter.format(wheelRadius) 
          + " meters, "
          + formatter.format(Units.metersToInches(wheelRadius))
          + " inches");
        })));
  }

  private static class WheelRadiusCharacterizationState {
    private double[] positions = new double[4];
    private Rotation2d lastAngle = new Rotation2d();
    private double gyroDelta = 0.0;
  }

  public static Command RobotOrientedDrive(
    Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
      return Commands.run(
        () -> {
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadband);

          omega = Math.copySign(omega * omega, omega);

          ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec());
          
            drive.runVelocity(speeds);
        },
        drive);
    }

  // Called when the command is initially scheduled.
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {}

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}