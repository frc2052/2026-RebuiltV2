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
    shootingTable.put(Meters.of(2.5), new Pair<>(RotationsPerSecond.of(34), Degrees.of(12)));
    shootingTable.put(Meters.of(3), new Pair<>(RotationsPerSecond.of(37), Degrees.of(12)));
    shootingTable.put(Meters.of(3.5), new Pair<>(RotationsPerSecond.of(41), Degrees.of(13.3)));
    shootingTable.put(Meters.of(4), new Pair<>(RotationsPerSecond.of(44), Degrees.of(14.9)));
    shootingTable.put(Meters.of(4.5), new Pair<>(RotationsPerSecond.of(48), Degrees.of(15.3)));
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
