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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgCommands;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.ReefAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algPivot.AlgPivotIO;
import frc.robot.subsystems.algPivot.AlgPivotIOFalcon;
import frc.robot.subsystems.algPivot.AlgPivotIOSim;
import frc.robot.subsystems.algPivot.AlgPivotSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOHardware;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.funnel.FunnelIO;
import frc.robot.subsystems.funnel.FunnelIOFalcon;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.vision.VisionIOHardwareLimelight;
import frc.robot.subsystems.vision.VisionIOSimPhoton;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final ElevatorSubsystem elevator;
  private final EndEffectorSubsystem endEffector;
  private final FunnelSubsystem funnel;
  private final AlgPivotSubsystem algPivot;

  private final VisionSubsystem vision;
  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
        elevator = new ElevatorSubsystem(new ElevatorIOKraken());
        endEffector = new EndEffectorSubsystem(new EndEffectorIOHardware());
        funnel = new FunnelSubsystem(new FunnelIOFalcon());

        algPivot = new AlgPivotSubsystem(new AlgPivotIOFalcon());
        vision = new VisionSubsystem(new VisionIOHardwareLimelight(drive));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        elevator = new ElevatorSubsystem(new ElevatorIOSim());
        endEffector = new EndEffectorSubsystem(new EndEffectorIOHardware());
        funnel = new FunnelSubsystem(new FunnelIOFalcon());

        algPivot = new AlgPivotSubsystem(new AlgPivotIOSim());
        vision = new VisionSubsystem(new VisionIOSimPhoton(drive));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new ElevatorSubsystem(new ElevatorIO() {});
        endEffector = new EndEffectorSubsystem(new EndEffectorIO() {});
        funnel = new FunnelSubsystem(new FunnelIO() {});

        algPivot = new AlgPivotSubsystem(new AlgPivotIO() {});
        vision = new VisionSubsystem(new VisionIOSimPhoton(drive));

        break;
    }

    // Configure the button bindings
    configureButtonBindings();

    NamedCommands.registerCommand(
        "EnablePoseEstimate", new InstantCommand(() -> VisionSubsystem.allowVisionUpdates = true));
    // NamedCommands.registerCommand(
    //   "DisablePoseEstimate",
    //  new InstantCommand(() -> VisionSubsystem.allowVisionUpdates = false));

    NamedCommands.registerCommand("ScoreCoral", getAutoScoreCommand(endEffector, elevator));
    NamedCommands.registerCommand(
        "PickupCoral", EndEffectorCommands.endEffectorSensorCommand(endEffector, 2.45));
    NamedCommands.registerCommand(
        "ElevatorLevelTwo", new InstantCommand(() -> elevator.setDistance(7.00)));
    NamedCommands.registerCommand(
        "ElevatorLevelThree", new InstantCommand(() -> elevator.setDistance(17.25)));
    NamedCommands.registerCommand(
        "ElevatorLevelFour", new InstantCommand(() -> elevator.setDistance(34.95)));
    NamedCommands.registerCommand(
        "ElevatorLevelOne",
        ElevatorCommands.setDistance(elevator, 0)
            .raceWith(Commands.waitUntil(() -> elevator.getDistance() <= 0.1)));
    NamedCommands.registerCommand("ReefAlignRight", new ReefAlign(vision, drive, false));
    NamedCommands.registerCommand("ReefAlignRight", new ReefAlign(vision, drive, true));
    NamedCommands.registerCommand(
        "ServoHome",
        new InstantCommand(
            () -> {
              funnel.setServoHome();
            }));
    NamedCommands.registerCommand(
        "PivotDown", AlgCommands.setPivotPosition(algPivot, -95).withTimeout(1));
    NamedCommands.registerCommand(
        "PivotUp", AlgCommands.setPivotPosition(algPivot, -15).withTimeout(1));

    NamedCommands.registerCommand(
        "ServoDisable",
        new InstantCommand(
            () -> {
              funnel.setServoDisable();
            }));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        new DefaultDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    new Trigger(() -> endEffector.getHasObject() == true)
        .onTrue(
            new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 1.0))
                .andThen(
                    new WaitCommand(1.0)
                        .andThen(
                            new InstantCommand(
                                () -> controller.setRumble(RumbleType.kBothRumble, 0)))));

    controller
        .R1()
        .whileTrue(
            EndEffectorCommands.setEndEffectorVolts(endEffector, 3.0)
                .alongWith(
                    new ConditionalCommand(
                        ElevatorCommands.setDistance(elevator, 35.95),
                        new InstantCommand(),
                        () -> elevator.getDistance() >= 35.0)));

    controller
        .L1()
        .whileTrue(
            EndEffectorCommands.endEffectorSensorCommand(endEffector, 2.8)
                .alongWith(
                    new InstantCommand(
                        () -> {
                          Drive.setTargetHeading(
                              DriverStation.getAlliance().get() == Alliance.Red
                                  ? Rotation2d.fromDegrees(127.0).times(Drive.leftSource ? 1 : -1)
                                  : Rotation2d.fromDegrees(53.0).times(Drive.leftSource ? -1 : 1));
                          Drive.setDriveState(DriveState.SNAP_HEADING);
                        }))
                .alongWith(ElevatorCommands.setDistance(elevator, 0.0)));

    controller
        .L2()
        .whileTrue(
            new ConditionalCommand(
                new ReefAlign(vision, drive, true),
                new InstantCommand(),
                () -> vision.getFrontCamTargetValid()));

    controller
        .R2()
        .whileTrue(
            new ConditionalCommand(
                new ReefAlign(vision, drive, false),
                new InstantCommand(),
                () -> vision.getFrontCamTargetValid()));

    controller.cross().whileTrue(ElevatorCommands.setDistance(elevator, 0.0));
    controller.square().whileTrue(ElevatorCommands.setDistance(elevator, 7.00));
    controller.circle().whileTrue(ElevatorCommands.setDistance(elevator, 17.25));
    controller.triangle().whileTrue(ElevatorCommands.setDistance(elevator, 35.0));

    controller.povUp().whileTrue(ElevatorCommands.setElevatorVolts(elevator, 2.0));
    controller.povDown().whileTrue(ElevatorCommands.setElevatorVolts(elevator, -1.0));

    controller.povLeft().onTrue(new InstantCommand(() -> Drive.leftSource = true));
    controller.povRight().onTrue(new InstantCommand(() -> Drive.leftSource = false));

    controller
        .button(10)
        .whileTrue(AlgCommands.setPivotPosition(algPivot, -95.0))
        .onFalse(AlgCommands.setPivotPosition(algPivot, -15.0));
  }

  /**
   * Use this
   *
   * <p>o pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Command getScoreCommand(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
    return EndEffectorCommands.setEndEffectorVolts(endEffector, 5.5)
        .raceWith(Commands.waitUntil(() -> !endEffector.getHasObject()))
        .andThen(new WaitCommand(0.35))
        .andThen(
            ElevatorCommands.setDistance(elevator, 0)
                .raceWith(Commands.waitUntil(() -> elevator.getDistance() <= 0.1)));
  }

  private Command getAutoScoreCommand(
      EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
    return EndEffectorCommands.setEndEffectorVolts(endEffector, 5.5)
        .raceWith(Commands.waitUntil(() -> !endEffector.getHasObject()))
        .andThen(new InstantCommand(() -> elevator.setDistance(0)));
  }
}
