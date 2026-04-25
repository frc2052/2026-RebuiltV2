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

  private static final double squishMod = 0.75;

  static {
    Map<Distance, Pair<AngularVelocity, Angle>> shootingTable = new HashMap<>();
    // Initialize the shooting table with distance, shooter velocity, and hood angle pairs
    shootingTable.put(
        Meters.of(1.5), new Pair<>(RotationsPerSecond.of(28.75 + squishMod), Degrees.of(3.6)));
    shootingTable.put(
        Meters.of(2), new Pair<>(RotationsPerSecond.of(29.6 + squishMod), Degrees.of(4.6)));
    shootingTable.put(
        Meters.of(2.5), new Pair<>(RotationsPerSecond.of(30 + squishMod), Degrees.of(6)));
    shootingTable.put(
        Meters.of(3), new Pair<>(RotationsPerSecond.of(31.15 + squishMod), Degrees.of(7.7)));
    shootingTable.put(
        Meters.of(3.5), new Pair<>(RotationsPerSecond.of(32.5 + squishMod), Degrees.of(11.26)));
    shootingTable.put(
        Meters.of(4), new Pair<>(RotationsPerSecond.of(33.75 + squishMod), Degrees.of(15.26)));
    shootingTable.put(
        Meters.of(4.5), new Pair<>(RotationsPerSecond.of(35.5 + squishMod), Degrees.of(19.76)));
    shootingTable.put(
        Meters.of(5), new Pair<>(RotationsPerSecond.of(36.75 + squishMod), Degrees.of(21.26)));

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
