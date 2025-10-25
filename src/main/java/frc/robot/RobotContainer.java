package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.Constants.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSpark;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    private final Drive drive;

    private final CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        switch(Constants.currentMode) {
            case REAL -> {
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3));
            }

            default -> {
                drive = new Drive(
                    new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
            }
        }
    }

    private void configureButtonBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // controller.a().whileTrue(DriveCommands.joystickDrive(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(),()->{return 0.0;}));

        controller.x().onTrue(Commands.runOnce(drive::StopWithX, drive));
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    public void resetPose(Pose2d pose) {
        drive.resetOdometry(pose);
    }
}

    
