package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.subsystems.CANCoderConstants;
import com.team2052.lib.subsystems.ServoSubsystemConstants;
import com.team2052.lib.subsystems.ServoSubsystemWithCANCoder;
import edu.wpi.first.units.measure.Angle;
import lombok.Getter;

public class HoodSubsystem extends ServoSubsystemWithCANCoder {

  private static HoodSubsystem INSTANCE;

  @Getter private Angle goalAngle = Rotations.of(0);
  private Angle lastGoalAngle = goalAngle;

  public static HoodSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new HoodSubsystem(HoodConstants.SERVO_CONSTANTS, HoodConstants.ENCODER_CONSTANTS);
    }
    return INSTANCE;
  }

  private HoodSubsystem(ServoSubsystemConstants constants, CANCoderConstants canCoderConstants) {
    super(constants, canCoderConstants);
    goalAngle = getPosition();
    lastGoalAngle = goalAngle;
  }

  public void setToAngle(Angle angle) {
    goalAngle = boundAngle(angle);
  }

  public boolean isAtAngle(Angle angle, Angle tolerance) {
    return MathHelpers.epsilonEquals(
        getPosition().in(Degrees), angle.in(Degrees), tolerance.in(Degrees));
  }

  private Angle boundAngle(Angle angle) {
    return Degrees.of(
        MathHelpers.clamp(
            angle.in(Degrees),
            HoodConstants.HOOD_MIN_ANGLE.in(Degrees),
            HoodConstants.HOOD_MAX_ANGLE.in(Degrees)));
  }

  @Override
  public void periodic() {
    super.periodic();
    if (goalAngle.in(Degrees) != lastGoalAngle.in(Degrees)) {
      setSetpointMotionMagic(goalAngle);
      lastGoalAngle = goalAngle;
    }
  }
}
