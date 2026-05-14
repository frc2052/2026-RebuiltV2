package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionConstants;

public class DrivetrainSubsystem extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005;
  private Notifier simNotifier = null;
  private double lastSimTime;
  private boolean hasAppliedOperatorPerspective = false;

  private ChassisSpeeds autoTargetSpeeds = new ChassisSpeeds();

  private final SwerveRequest.ApplyRobotSpeeds autoSwerveRequest =
      new SwerveRequest.ApplyRobotSpeeds();

  private static DrivetrainSubsystem INSTANCE;

  public static DrivetrainSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new DrivetrainSubsystem();
    }

    return INSTANCE;
  }

  private DrivetrainSubsystem() {
    super(
        DrivetrainConstants.TUNER_DRIVETRAIN_CONSTANTS.getDrivetrainConstants(),
        DrivetrainConstants.ODOMETRY_FREQUENCY,
        DrivetrainConstants.ODOMETRY_STDDEV,
        VisionConstants.VISION_STDDEV,
        DrivetrainConstants.TUNER_DRIVETRAIN_CONSTANTS.getModuleConstants());

    if (Robot.isSimulation()) {
      startSimThread();
    }

    GyroTrimConfigs trim = new GyroTrimConfigs().withGyroScalarZ(0.869); // -3.263

    getPigeon2().getConfigurator().apply(trim);

    CommandScheduler.getInstance().registerSubsystem(this);
    configureAutoBuilder();
  }

  public void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Current Pose supplier
          this::resetPose, // Consumer, seeds pose agains auto
          () -> getState().Speeds, // Current Speeds Supplier
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) -> {
            autoTargetSpeeds = speeds;
            setControl(
                autoSwerveRequest
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
          },
          new PPHolonomicDriveController(
              // TODO: tune auto PIDs
              new PIDConstants(5, 0, 0), new PIDConstants(3.5, 0, 0)),
          config,
          () -> (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red),
          this); // Drivetrain Subsystem for requirements
    } catch (Exception e) {
      DriverStation.reportError(
          "====== Failed to load PathPlanner config and configure AutoBuilder.", e.getStackTrace());
    }
  }

  public void stop() {
    setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
  }

  private Vector<N3> getOdometryStdDevs() {

    double stdDev = Double.MAX_VALUE;
    double thetaDev = 0; // trust the gyro fully

    // LinearAcceleration xAccel = getPigeon2().getAccelerationX().getValue();
    // LinearAcceleration yAccel = getPigeon2().getAccelerationY().getValue();
    // LinearAcceleration zAccel = getPigeon2().getAccelerationZ().getValue();
    // LinearAcceleration totalAcceleration =
    //     MetersPerSecondPerSecond.of(
    //             Math.hypot(
    //                 xAccel.abs(MetersPerSecondPerSecond),
    //                 Math.hypot(
    //                     yAccel.abs(MetersPerSecondPerSecond),
    //                     zAccel.abs(MetersPerSecondPerSecond))))
    //         .minus(
    //             MetersPerSecondPerSecond.of(
    //                 9.8)); // minus cause gravity (it hovers at ~9.8 otherwise)

    // use for tuning the cutoff
    // Logger.recordOutput("Total Acceleration", totalAcceleration.in(MetersPerSecondPerSecond));

    double accelStdDev = 1;
    // if (totalAcceleration.abs(MetersPerSecondPerSecond)
    //     > 1.5) { // if we slam into something trust the odometry less
    //   accelStdDev = totalAcceleration.abs(MetersPerSecondPerSecond) * 3;
    // }

    Angle pitch = getPigeon2().getPitch().getValue();
    Angle roll = getPigeon2().getRoll().getValue();

    // Use for tuning cutoff and scaling of angle std dev
    // Logger.recordOutput("Pitch", pitch.in(Degrees));
    // Logger.recordOutput("Roll", roll.in(Degrees));
    // Logger.recordOutput("Pitch + Roll", pitch.plus(roll).abs(Degrees));

    double angleStdDev = 1;
    if (pitch.plus(roll).abs(Degrees)
        > 5) { // if we're tilted significantly, trust the odometry less
      angleStdDev = pitch.plus(roll).abs(Degrees) * 2;
    }

    stdDev = (DrivetrainConstants.ODOMETRY_STDDEV_COEFFICIENT * accelStdDev * angleStdDev) + 0.01;

    return VecBuilder.fill(stdDev, stdDev, thetaDev);
  }

  @Override
  public void periodic() {
    super.setStateStdDevs(getOdometryStdDevs());

    /* Periodically try to apply the operator perspective
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? Rotation2d.fromDegrees(180)
                        : Rotation2d.fromDegrees(0));
                hasAppliedOperatorPerspective = true;
              });
    }
    RobotState.getInstance().setDrivetrainState(getStateCopy());
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  @SuppressWarnings("unused")
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  @SuppressWarnings("unused")
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }
}
