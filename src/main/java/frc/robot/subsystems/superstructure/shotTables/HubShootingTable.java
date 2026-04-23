package frc.robot.subsystems.superstructure.shotTables;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.*;
import java.util.HashMap;
import java.util.Map;

public class HubShootingTable extends ShootingTableBase {

  private static HubShootingTable INSTANCE;

  static {
    Map<Distance, Pair<AngularVelocity, Angle>> shootingTable = new HashMap<>();
    // Initialize the shooting table with distance, shooter velocity, and hood angle pairs
    shootingTable.put(Meters.of(1.5), new Pair<>(RotationsPerSecond.of(28), Degrees.of(2)));
    shootingTable.put(Meters.of(2), new Pair<>(RotationsPerSecond.of(30), Degrees.of(2.5)));
    shootingTable.put(Meters.of(2.5), new Pair<>(RotationsPerSecond.of(31), Degrees.of(3)));
    shootingTable.put(Meters.of(3), new Pair<>(RotationsPerSecond.of(32), Degrees.of(4)));
    shootingTable.put(Meters.of(3.5), new Pair<>(RotationsPerSecond.of(31), Degrees.of(6.6)));
    shootingTable.put(Meters.of(4), new Pair<>(RotationsPerSecond.of(35), Degrees.of(7.1)));
    shootingTable.put(Meters.of(4.5), new Pair<>(RotationsPerSecond.of(37.5), Degrees.of(8)));
    // Add more entries as needed
    INSTANCE = new HubShootingTable(shootingTable);
  }

  public static HubShootingTable getInstance() {
    return INSTANCE;
  }

  private HubShootingTable(Map<Distance, Pair<AngularVelocity, Angle>> shootingTable) {
    super(shootingTable);
  }
}
