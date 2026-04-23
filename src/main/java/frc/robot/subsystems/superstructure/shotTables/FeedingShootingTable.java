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
    shootingTable.put(Meters.of(3.2), new Pair<>(RotationsPerSecond.of(28), Degrees.of(12)));
    shootingTable.put(Meters.of(6.6), new Pair<>(RotationsPerSecond.of(47), Degrees.of(18)));
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
