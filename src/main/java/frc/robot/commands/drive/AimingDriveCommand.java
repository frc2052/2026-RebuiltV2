package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.TargetType;
import frc.robot.util.ShotProfile;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AimingDriveCommand extends SnapToAngleCommand {

  private BooleanSupplier lockWheelsSupplier;

  public AimingDriveCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentricSupplier,
      BooleanSupplier useSOTMSupplier,
      BooleanSupplier lockWheelsSupplier,
      Optional<TargetType> optForcedTargetType) {
    super(
        () -> getDesiredDirection(useSOTMSupplier, optForcedTargetType),
        xSupplier,
        ySupplier,
        rotationSupplier,
        fieldCentricSupplier);
    this.lockWheelsSupplier = lockWheelsSupplier;
  }

  protected static Rotation2d getDesiredDirection(
      BooleanSupplier useSOTMSupplier, Optional<TargetType> optForcedTargetType) {
    boolean useSOTM = useSOTMSupplier.getAsBoolean();
    Superstructure superstructure = Superstructure.getInstance();

    ShotProfile profile;
    if ((useSOTM && superstructure.isShootOnTheMove())
        || (!useSOTM && !superstructure.isShootOnTheMove())) {
      // Ensure the shot profile is up to date before getting the aiming parameters
      if (optForcedTargetType.isPresent()) {
        superstructure.calculateShotProfile(optForcedTargetType.get());
      } else {
        superstructure.calculateShotProfile();
      }
      profile = superstructure.getLastCalculatedProfile();
    } else {
      superstructure.setShootOnTheMove(
          useSOTM); // Update the SOTM state if it doesn't match the supplier
      // Recalculate the shot profile with the updated SOTM state
      if (optForcedTargetType.isPresent()) {
        superstructure.recalculateShotProfile(optForcedTargetType.get());
      } else {
        superstructure.recalculateShotProfile();
      }
      profile = superstructure.getLastCalculatedProfile();
    }

    if (profile == null) {
      return RobotState.getInstance().getFieldToRobot().getRotation();
    }

    return profile.aimingParameters.robotRotation;
  }

  @Override
  protected SwerveRequest getSwerveRequest() {
    if (lockWheelsSupplier.getAsBoolean()) {
      return new SwerveRequest.SwerveDriveBrake();
    } else {
      return super.getSwerveRequest();
    }
  }
}
