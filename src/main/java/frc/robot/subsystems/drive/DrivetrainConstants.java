package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.drive.ctre.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import frc.robot.util.FieldConstants;

public final class DrivetrainConstants {
    public static final double ODOMETRY_FREQUENCY = 100.0; // Hz
    public static final CommandSwerveDrivetrain TUNER_DRIVETRAIN_CONSTANTS =
        TunerConstants.createDrivetrain();
    public static final double ODOMETRY_STDDEV_COEFFICIENT = 0.02;
    public static final Matrix<N3, N1> ODOMETRY_STDDEV =
        new Matrix<>(VecBuilder.fill(0.01, 0.01, 0.01));

    public static final Angle HEADING_TOLERANCE = Degrees.of(3);
    public static final Translation2d SEED_RIGHT_RED_TRANSLATION =
        new Translation2d(FieldConstants.fieldLength - 0.5, FieldConstants.fieldWidth - 0.5);
    public static final Translation2d SEED_LEFT_RED_TRANSLATION =
        new Translation2d(FieldConstants.fieldLength - 0.5, 0.5);
    public static final Translation2d SEED_RIGHT_BLUE_TRANSLATION = new Translation2d(0.5, 0.5);
    public static final Translation2d SEED_LEFT_BLUE_TRANSLATION =
        new Translation2d(0.5, FieldConstants.fieldWidth - 0.5);
}
