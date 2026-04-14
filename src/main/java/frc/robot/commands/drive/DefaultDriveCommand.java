// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {

  protected final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier fieldCentricSupplier;

  protected final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond);

  protected final SwerveRequest.FieldCentric driveNoHeading =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.05) // Add a 5% deadband in open loop
          .withRotationalDeadband(maxAngularRate * 0.05)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  protected final SwerveRequest.ApplyFieldSpeeds driveChassisSpeeds =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDesaturateWheelSpeeds(true)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  protected final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric()
          .withDeadband(maxSpeed * 0.05)
          .withRotationalDeadband(maxAngularRate * 0.05) // Add a 5% deadband
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  protected SwerveRequest.FieldCentricFacingAngle driveWithHeading =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(maxSpeed * 0.05)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  /**
   * @param xSupplier supplier for forward velocity.
   * @param ySupplier supplier for sideways velocity.
   * @param rotationSupplier supplier for angular velocity.
   */
  public DefaultDriveCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentricSupplier) {

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldCentricSupplier = fieldCentricSupplier;

    driveWithHeading.HeadingController.setPID(2.5, 0.0, 0.0);
    driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  protected double getX() {
    return -MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.05);
  }

  protected double getY() {
    return -MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.05);
  }

  protected double getRotation() {
    return -MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.05);
  }

  protected boolean getFieldCentric() {
    return fieldCentricSupplier.getAsBoolean();
  }

  protected SwerveRequest getSwerveRequest() {
    if (getFieldCentric()) {
      return driveNoHeading
          .withVelocityX(getX() * maxSpeed)
          .withVelocityY(getY() * maxSpeed)
          .withRotationalRate(getRotation() * maxAngularRate);
    } else {
      return robotCentricDrive
          .withVelocityX(getX())
          .withVelocityY(getY())
          .withRotationalRate(getRotation());
    }
  }

  @Override
  public void execute() {
    drivetrain.setControl(getSwerveRequest());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
