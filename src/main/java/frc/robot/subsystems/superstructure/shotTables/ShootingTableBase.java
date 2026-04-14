package frc.robot.subsystems.superstructure.shotTables;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;
import java.util.Map;

public class ShootingTableBase {

  private final Map<Distance, Pair<AngularVelocity, Angle>> shootingTable;
  private final InterpolatingDoubleTreeMap angularVelocityMap;
  private final InterpolatingDoubleTreeMap angleMap;

  public ShootingTableBase(Map<Distance, Pair<AngularVelocity, Angle>> shootingTable) {
    this.shootingTable = shootingTable;
    this.angularVelocityMap = new InterpolatingDoubleTreeMap();
    this.angleMap = new InterpolatingDoubleTreeMap();
    for (Map.Entry<Distance, Pair<AngularVelocity, Angle>> entry : shootingTable.entrySet()) {
      Distance distance = entry.getKey();
      AngularVelocity shooterVelocity = entry.getValue().getFirst();
      Angle hoodAngle = entry.getValue().getSecond();
      angularVelocityMap.put(distance.in(Meters), shooterVelocity.in(RotationsPerSecond));
      angleMap.put(distance.in(Meters), hoodAngle.in(Degrees));
    }
  }

  public Map<Distance, Pair<AngularVelocity, Angle>> getShootingTable() {
    return shootingTable;
  }

  public Pair<AngularVelocity, Angle> getShootingParameters(Distance distance) {
    double distanceMeters = distance.in(Meters);
    double shooterVelocityRPS = angularVelocityMap.get(distanceMeters);
    double hoodAngleDegrees = angleMap.get(distanceMeters);
    return new Pair<>(RotationsPerSecond.of(shooterVelocityRPS), Degrees.of(hoodAngleDegrees));
  }
}
