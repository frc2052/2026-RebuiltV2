package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.Pair;

public class Ports {
  /*
   * CAN:
   * A Pair holds both CAN ID and CAN Bus for devices
   * When creating a device, use .getFirst() for ID and .getSecond() for Bus
   */
  public static final CANBus MAIN_BUS = new CANBus("NotDriveCAN");
  public static final CANBus RIO_BUS = CANBus.roboRIO();

  public static final Pair<Integer, CANBus> FLOOR_LEFT_MOTOR = Pair.of(34, MAIN_BUS);
  public static final Pair<Integer, CANBus> FLOOR_RIGHT_MOTOR = Pair.of(33, MAIN_BUS);

  public static final Pair<Integer, CANBus> SHOOTER_TOP_LEFT = Pair.of(25, MAIN_BUS);
  public static final Pair<Integer, CANBus> SHOOTER_BOTTOM_LEFT = Pair.of(26, MAIN_BUS);
  public static final Pair<Integer, CANBus> SHOOTER_TOP_RIGHT = Pair.of(23, MAIN_BUS);
  public static final Pair<Integer, CANBus> SHOOTER_BOTTOM_RIGHT = Pair.of(22, MAIN_BUS);

  public static final Pair<Integer, CANBus> STAGER_TOP_MOTOR = Pair.of(21, MAIN_BUS);
  public static final Pair<Integer, CANBus> STAGER_BOTTOM_MOTOR = Pair.of(20, MAIN_BUS);

  public static final Pair<Integer, CANBus> HOOD_MOTOR = Pair.of(27, MAIN_BUS);
  public static final Pair<Integer, CANBus> HOOD_ENCODER = Pair.of(24, MAIN_BUS);

  public static final Pair<Integer, CANBus> LEFT_INTAKE_MOTOR = Pair.of(30, MAIN_BUS);
  public static final Pair<Integer, CANBus> RIGHT_INTAKE_MOTOR = Pair.of(31, MAIN_BUS);

  public static final Pair<Integer, CANBus> INTAKE_ENCODER = Pair.of(32, MAIN_BUS);
  public static final Pair<Integer, CANBus> INTAKE_PIVOT_MOTOR = Pair.of(29, MAIN_BUS);

  public static final Pair<Integer, CANBus> HOPPER_EXTENDER_MOTOR = Pair.of(28, MAIN_BUS);
}
