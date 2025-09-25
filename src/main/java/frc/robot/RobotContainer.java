package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class RobotContainer extends LoggedRobot {
    private final CommandXboxController controller = new CommandXboxController(0);
    // Subsystems
    private final Drive drive;
    public final Pivot pivot = new Pivot();
    private final Vision vision;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));
                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOLimelight(camera0Name, drive::getRotation),
                                new VisionIOLimelight(camera1Name, drive::getRotation));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                        new Drive(
                                new GyroIO() {
                                },
                                new ModuleIOSim(TunerConstants.FrontLeft),
                                new ModuleIOSim(TunerConstants.FrontRight),
                                new ModuleIOSim(TunerConstants.BackLeft),
                                new ModuleIOSim(TunerConstants.BackRight));
                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new Drive(
                                new GyroIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                });
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        //        configureSwerveBindings();
        controller.a().whileTrue(pivot.setThenRunState(PivotConstants.PivotStates.IDLE));
        controller.b().whileTrue(pivot.setThenRunState(PivotConstants.PivotStates.UP));
    }

    private void configureSwerveBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                Rotation2d::new));

        // Switch to an X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                        () ->
                                                drive.setPose(
                                                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                        drive)
                                .ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
