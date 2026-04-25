package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.MatchState;

/**
 * Subsystem to control the robot's LEDs, by determining what number should be encoded to DIO pins
 * and sent to the Arduino we used for controlling the patterns and colors
 */
public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem INSTANCE;
  public static final int LED_CHANNEL_1_PIN = 0;
  public static final int LED_CHANNEL_2_PIN = 1;
  public static final int LED_CHANNEL_3_PIN = 2;
  private final DigitalOutput codeChannel1, codeChannel2, codeChannel3;
  private final RobotState state;

  private LEDStatusMode currentStatusMode;

  private boolean disableLEDs;
  private boolean robotDisabled;

  // this needs to be updated, and any references to these.
  private LEDSubsystem() {
    // DIO outputs
    codeChannel1 = new DigitalOutput(LED_CHANNEL_1_PIN);
    codeChannel2 = new DigitalOutput(LED_CHANNEL_2_PIN);
    codeChannel3 = new DigitalOutput(LED_CHANNEL_3_PIN);
    // codeChannel4 = new DigitalOutput(Ports.LED_CHANNEL_4_PIN);
    // codeChannel5 = new DigitalOutput(Ports.LED_CHANNEL_5_PIN);
    robotDisabled = false;

    currentStatusMode = LEDStatusMode.OFF;

    state = RobotState.getInstance();

    // For manually inputing code to encode to DIO pins
    // SmartDashboard.putNumber("LED CODE", 0);
  }

  public static LEDSubsystem
      getInstance() { // Method to allow calling this class and getting the single instance from
    // anywhere, creating the instance if the first time.
    if (INSTANCE == null) {
      INSTANCE = new LEDSubsystem();
    }
    return INSTANCE;
  }

  // this needs to be updates
  public static enum LEDStatusMode {
    OFF(0),
    HUB_TRANSITION_BLUE(1),
    HUB_ACTIVE_BLUE(2),
    HUB_TRANSITION_RED(3),
    HUB_ACTIVE_RED(4),
    IDLE(5);

    private final int code;

    private LEDStatusMode(int code) {
      this.code = code;
    }

    public int getPositionTicks() {
      return code;
    }
  }

  @Override
  public void periodic() {
    int code = 0;
    if (!disableLEDs) {
      if (DriverStation.isAutonomousEnabled()) {
        if ((DriverStation.getAlliance().get() == Alliance.Red)) {
          currentStatusMode = LEDStatusMode.HUB_ACTIVE_RED;
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
          currentStatusMode = LEDStatusMode.HUB_ACTIVE_BLUE;
        }
        // if (currentStatusMode.code != code) {
        code = currentStatusMode.code;
        // }
      }
      /*if (DriverStation.isDisabled()) {
          // If disabled, gets the alliance color from the driver station and pulses that. Only pulses
          // color if connected to station or FMS, else pulses default disabled color (Firefl status
          // mode)
          AutoChooser.getAuto() selected = .getInstance().getAuto();
          if (selected == Auto.NO_AUTO || selected == null) {
              currentStatusMode = LEDStatusMode.NO_AUTO;
          } // Reaches here if DriverStation.getAlliance returns Invalid, which just
          // means it can't determine our alliance and we do cool default effect

          // autonomous LED status modes

      } else if (DriverStation.isAutonomous()) {
          // example implementation

          // if(RobotState.getInstance().getNoteHeldDetected()){
          //     currentStatusMode = LEDStatusMode.Has_Coral;
          // } else if (RobotState.getInstance().getIsShamperAtGoalAngle()){
          //     // currentStatusMode = LEDStatusMode.A;
          // } else {

          // }

      }*/

      // teleop LED status modes

      else if (DriverStation.isTeleopEnabled()) {
        if ((DriverStation.getAlliance().get() == Alliance.Red)) {
          if (MatchState.getTimeToShiftChange().in(Seconds) <= 3) {
            currentStatusMode = LEDStatusMode.HUB_TRANSITION_RED;
          } else if (MatchState.isHubActive()) {
            currentStatusMode = LEDStatusMode.HUB_ACTIVE_RED;
          } else {
            currentStatusMode = LEDStatusMode.OFF;
          }
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
          if (MatchState.getTimeToShiftChange().in(Seconds) <= 3) {
            currentStatusMode = LEDStatusMode.HUB_TRANSITION_BLUE;
          } else if (MatchState.isHubActive()) {
            currentStatusMode = LEDStatusMode.HUB_ACTIVE_BLUE;
          } else {
            currentStatusMode = LEDStatusMode.OFF;
          }
        }

        // } else if (Superstructure.getInstance().isFiring())
        // } else if (Superstructure.getInstance().getCurrentState() == ScoringState.HUB) {
        //     currentStatusMode = LEDStatusMode.HUB;
        // } else if (Superstructure.getInstance().getCurrentState() ==
        // ScoringState.FEEDING_DEPOT_SIDE) {
        //     currentStatusMode = LEDStatusMode.FEEDING_DEPOT_SIDE;
        // } else if (Superstructure.getInstance().getCurrentState() ==
        // ScoringState.FEEDING_OUTPOST_SIDE) {
        //  currentStatusMode = LEDStatusMode.FEEDING_OUTPOST_SIDE;
        // }

        // shooting
        // if(RobotState.getInstance().getShooting()){
        //     if(!RobotState.getInstance().getNoteHeldDetected()){
        //         currentStatusMode = LEDStatusMode.DANGER;
        //                         } else if (RobotState.getInstance().getNoteHeldDetected() &&
        // RobotState.getInstance().getIsShamperAtGoalAngle() &&
        // RobotState.getInstance().getIsRotationOnTarget()){
        //         currentStatusMode = LEDStatusMode.SHOOTING_ON_TARGET;
        //     } else {
        //         currentStatusMode = LEDStatusMode.SHOOTING;
        //     }
        // }
        // //  aimed
        // else if (RobotState.getInstance().getNoteHeldDetected() &&
        // RobotState.getInstance().getIsShamperAtGoalAngle() &&
        // RobotState.getInstance().getIsRotationOnTarget()){
        //     currentStatusMode = LEDStatusMode.AIMING_ON_TARGET;
        // }
        // // aiming
        // else if (RobotState.getInstance().getIsVerticalAiming() ||
        // RobotState.getInstance().getIsHorizontalAiming())
        // {
        //     currentStatusMode = LEDStatusMode.AIMING;
        // } else if (RobotState.getInstance().getAmpIdle()){
        //     currentStatusMode = LEDStatusMode.AMP_IDLE;
        // }
        // else {
        //     if(RobotState.getInstance().getNoteHeldDetected()){
        //         currentStatusMode = LEDStatusMode.Has_Coral;
        //     } else if (RobotState.getInstance().getIsIntaking()){
        //         currentStatusMode = LEDStatusMode.INTAKE;
        //     } else {
        //         currentStatusMode = LEDStatusMode.OFF;
        //     }
        // }

        // }
      } else {
        currentStatusMode = LEDStatusMode.IDLE;
      }
      // update new code
      // if (currentStatusMode.code != code) {
      code = currentStatusMode.code;
      // }
    } else {
      // LEDs are disabled
      code = 0;
    }
    System.out.println(currentStatusMode);
    // Code for encoding the code to binary on the digitalOutput pins
    // RobotState.getInstance().putData("Sending LED Code", code);
    codeChannel1.set((code & 1) > 0); // 2^0
    codeChannel2.set((code & 2) > 0); // 2^1
    codeChannel3.set((code & 4) > 0); // 2^2
    // codeChannel4.set((code & 8) > 0); // 2^3
    // codeChannel5.set((code & 16) > 0); // 2^4
  }

  public void setLEDStatusMode(LEDStatusMode statusMode) {
    if (!disableLEDs) {
      currentStatusMode = statusMode;
    }
  }

  public LEDStatusMode getLEDStatusMode() {
    return currentStatusMode;
  }

  public void clearStatusMode() {
    currentStatusMode = LEDStatusMode.OFF;
  }

  // Disables LEDs (turns them off)
  public void disableLEDs() {
    disableLEDs = true;
  }

  // Enables LEDs (turns them on)
  public void enableLEDs() {
    disableLEDs = false;
  }

  public boolean getRobotDisabled() {
    return robotDisabled;
  }
}
