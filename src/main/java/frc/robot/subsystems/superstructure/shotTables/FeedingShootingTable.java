package frc.robot.subsystems.superstructure.shotTables;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import java.util.HashMap;
import java.util.Map;

public class FeedingShootingTable extends ShootingTableBase {

  private static FeedingShootingTable INSTANCE;

  static {
    Map<Distance, Pair<AngularVelocity, Angle>> shootingTable = new HashMap<>();
    // Initialize the shooting table with distance, shooter velocity, and hood angle pairs
    shootingTable.put(Meters.of(3), new Pair<>(RotationsPerSecond.of(30), Degrees.of(25.3)));
    shootingTable.put(Meters.of(4.5), new Pair<>(RotationsPerSecond.of(37), Degrees.of(25.3)));
    shootingTable.put(Meters.of(6), new Pair<>(RotationsPerSecond.of(42), Degrees.of(35.3)));
    shootingTable.put(Meters.of(8), new Pair<>(RotationsPerSecond.of(50), Degrees.of(35.8)));
    shootingTable.put(Meters.of(11.3), new Pair<>(RotationsPerSecond.of(62), Degrees.of(38.8)));

    // Add more entries as needed
    INSTANCE = new FeedingShootingTable(shootingTable);
  }

  public static FeedingShootingTable getInstance() {
    return INSTANCE;
  }

  private FeedingShootingTable(Map<Distance, Pair<AngularVelocity, Angle>> shootingTable) {
    super(shootingTable);
  }
}
