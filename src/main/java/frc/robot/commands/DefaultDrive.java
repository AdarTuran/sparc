// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDrive extends Command {

  private static final double DEADBAND = 0.1;
  private final Drive drive;
  private final DoubleSupplier xSupplier, ySupplier, omegaSupplier;
  private boolean isFlipped = false;
  private static final double ANGLE_KP = 5.0; // TODO: Requires tuning
  private static final double ANGLE_KD = 0.15; // TODO: Requires tuning
  private static final double ANGLE_MAX_VELOCITY = 8.0; // TODO: Requires tuning
  private static final double ANGLE_MAX_ACCELERATION = 25.0; // TODO: Requires tuning
  private static final double ANGLE_DEADBAND = 0.1;

  private static final double MAINTAIN_KP = 1.0; // TODO: Requires tuning
  private static final double MAINTAIN_KD = 0.0; // TODO: Requires tuning

  private PIDController angleController = new PIDController(ANGLE_KP, 0.0, ANGLE_KD);

  private final PIDController maintainController = new PIDController(MAINTAIN_KP, 0.0, MAINTAIN_KD);

  private double targetHeading = 0.0;
  private double oldTargetHeading = 0.0;
  private boolean targetHeadingChanged = false;
  private double maintainTarget;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.reset();
    angleController.setTolerance(Math.toRadians(1.5));
    maintainTarget = drive.getRotation().getRadians();
    maintainController.enableContinuousInput(-Math.PI, Math.PI);
    maintainController.setTolerance(Math.toRadians(1.5));
    maintainController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveState state = drive.getDriveState();
    Logger.recordOutput("DriveState", state);
    Logger.recordOutput("MaintainTarget", Math.toDegrees(maintainTarget));
    Logger.recordOutput("CurrentHeading", Math.toDegrees(drive.getRotation().getRadians()));
    targetHeading = drive.getTargetHeading().getRadians();

    targetHeadingChanged = Math.toDegrees(Math.abs(targetHeading - oldTargetHeading)) > 1.0;

    if (targetHeadingChanged) {
      angleController.reset();
    }

    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    if (ElevatorSubsystem.atL4) {
      linearVelocity = linearVelocity.times(0.2);
    }

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    ChassisSpeeds speeds;
    switch (state) {
      case JOYSTICK_OPENLOOP:
      default:
        // Convert to field relative speeds & send command

        if (Math.abs(omegaSupplier.getAsDouble()) < 0.1) {
          maintainTarget = drive.getRotation().getRadians();
          Drive.setDriveState(DriveState.MAINTAIN_HEADING);
        }

        omega = MathUtil.applyDeadband(omega, ANGLE_DEADBAND);
        speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec());

        isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        drive.runOpenLoop(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getRotation()));
        break;

      case JOYSTICK_VELOCITY:
        if (Math.abs(omegaSupplier.getAsDouble()) < 0.1) {
          maintainTarget = drive.getRotation().getRadians();
        }

        // Convert to field relative speeds & send command
        speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec());
        isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getRotation()));
        break;

      case SNAP_HEADING:
        omega = angleController.calculate(drive.getRotation().getRadians(), targetHeading);

        if (Math.abs(omegaSupplier.getAsDouble()) >= 0.15) {
          Drive.setDriveState(DriveState.JOYSTICK_OPENLOOP);
        }

        speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega);
        isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        drive.runOpenLoop( // TODO: Might want to change to velocity according to our default drive
            // style
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getRotation()));

        break;

      case MAINTAIN_HEADING:
        if (Math.abs(omegaSupplier.getAsDouble()) >= 0.1) {
          Drive.setDriveState(DriveState.JOYSTICK_OPENLOOP);
          break;
        }

        omega = maintainController.calculate(drive.getRotation().getRadians(), maintainTarget);
        omega = MathUtil.applyDeadband(omega, ANGLE_DEADBAND);

        speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega);
        isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        drive.runOpenLoop( // TODO: Might want to change to velocity according to our default drive
            // style
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getRotation()));
        break;

      case ORBIT_REEF:
        Rotation2d reefTarget =
            drive.getPose().getTranslation().minus(FieldConstants.redReefCenterPos).getAngle();
        SmartDashboard.putNumber("ReefTarget", reefTarget.getDegrees());
        omega =
            angleController.calculate(drive.getRotation().getRadians(), reefTarget.getRadians());
        omega = MathUtil.applyDeadband(omega, ANGLE_DEADBAND);

        speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega);
        isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        drive.runOpenLoop( // TODO: Might want to change to velocity according to our default drive
            // style
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getRotation()));
        break;
    }

    oldTargetHeading = targetHeading;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drive.setDriveState(DriveState.JOYSTICK_OPENLOOP);
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
