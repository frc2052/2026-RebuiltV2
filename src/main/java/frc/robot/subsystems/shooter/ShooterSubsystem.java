package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.team2052.lib.subsystems.RollerSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.Setter;

public class ShooterSubsystem extends RollerSubsystem {
  private static ShooterSubsystem INSTANCE;

  @Getter @Setter private AngularVelocity goalPoint = RotationsPerSecond.of(0);
  private AngularVelocity lastGoal = RotationsPerSecond.of(0);
  @Getter @Setter private boolean runningOpenLoop = false;

  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(1, 0.25));

  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterSubsystem();
    }
    return INSTANCE;
  }

  public ShooterSubsystem() {
    super(ShooterConstants.ROLLER_CONFIG);
  }

  @Override
  public Command runAtVelocityCommand(AngularVelocity velocity) {
    return Commands.runEnd(() -> setGoalPoint(velocity), () -> stopMotor(), this);
  }

  @Override
  public void stopMotor() {
    goalPoint = RotationsPerSecond.of(0);
    lastGoal = RotationsPerSecond.of(0);
    runningOpenLoop = false;
    super.stopMotor();
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("Goal Point", goalVelocity.in(RotationsPerSecond));
    SmartDashboard.putNumber("Shooter Speed", getVelocity().in(RotationsPerSecond));

    // if (goalPoint.in(RotationsPerSecond) != lastGoal.in(RotationsPerSecond)) {
    //   runningOpenLoop = true;

    //   lastGoal = goalPoint;
    // }
    if (goalPoint.in(RotationsPerSecond) == 0) {
      setCoastOut();
    }
    ;
    setGoalVelocityMotionMagic(goalPoint);

    // if ( // running open loop and within bounds
    // runningOpenLoop
    //     && MathHelpers.epsilonEquals(
    //         getVelocity().in(RotationsPerSecond),
    //         goalPoint.in(RotationsPerSecond),
    //         ShooterConstants.PID_USE_TOLERANCE.in(RotationsPerSecond))) {
    //   setGoalVelocityMotionMagic(goalPoint);
    //   runningOpenLoop = false;

    // } else if ( // not running open loop but not within bounds
    // !runningOpenLoop
    //     && !MathHelpers.epsilonEquals(
    //         getVelocity().in(RotationsPerSecond),
    //         goalPoint.in(RotationsPerSecond),
    //         ShooterConstants.PID_USE_TOLERANCE.in(RotationsPerSecond))) {

    //   runningOpenLoop = true;
    //   setOpenLoop(goalPoint.in(RotationsPerSecond) / 96);
    //   // trapezoidProfile.calculate(
    //   //         Constants.MAIN_LOOP_PERIOD.in(Seconds),
    //   //         new TrapezoidProfile.State(
    //   //             getVelocity().in(RotationsPerSecond) / 96,
    //   //             leader.getAcceleration().getValue().in(RotationsPerSecondPerSecond) / 96),
    //   //         new TrapezoidProfile.State(goalPoint.in(RotationsPerSecond) / 96, 0))
    //   //     .position);
    // } else if ( // running open loop and not in bounds
    // runningOpenLoop
    //     && !MathHelpers.epsilonEquals(
    //         getVelocity().in(RotationsPerSecond),
    //         goalPoint.in(RotationsPerSecond),
    //         ShooterConstants.PID_USE_TOLERANCE.in(RotationsPerSecond))) {
    //   // System.out.println("Running open loop : " + goalPoint.in(RotationsPerSecond) / 96);
    //   // + trapezoidProfile.calculate(
    //   //         Constants.MAIN_LOOP_PERIOD.in(Seconds),
    //   //         new TrapezoidProfile.State(
    //   //             getVelocity().in(RotationsPerSecond) / 96,
    //   //             leader.getAcceleration().getValue().in(RotationsPerSecondPerSecond) / 96),
    //   //         new TrapezoidProfile.State(goalPoint.in(RotationsPerSecond) / 96, 0))
    //   //     .position);
    //   setOpenLoop(goalPoint.in(RotationsPerSecond) / 96);
    //   // trapezoidProfile.calculate(
    //   //         Constants.MAIN_LOOP_PERIOD.in(Seconds),
    //   //         new TrapezoidProfile.State(
    //   //             getVelocity().in(RotationsPerSecond) / 96,
    //   //             leader.getAcceleration().getValue().in(RotationsPerSecondPerSecond) / 96),
    //   //         new TrapezoidProfile.State(goalPoint.in(RotationsPerSecond) / 96, 0))
    //   //     .position);
    // }
  }
}
