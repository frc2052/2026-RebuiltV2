package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;

public class OdometryFilter {

    @Getter private Pose2d currentPose = new Pose2d();
    private Time lastPoseTimestamp = Seconds.of(0);

    // NOTE: These poses should never directly alter the current pose, they are only for 
    private Pose2d lastWheelOdomPose = new Pose2d();
    private Pose2d lastQuestOdomPose = new Pose2d();

    private static OdometryFilter INSTANCE;

    public OdometryFilter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new OdometryFilter();
        }
        return INSTANCE;
    }

    private OdometryFilter() {

    }

    public void seedPose(Pose2d newPose) {
        currentPose = newPose;
        lastPoseTimestamp = Seconds.of(Timer.getFPGATimestamp());
    }

    public void update(
        Pair<Pose2d, Matrix<N3, N1>>[] visionMeasurementArray,
        Pair<Pose2d, Matrix<N3, N1>> wheelOdometryPose, 
        Pair<Pose2d, Matrix<N3, N1>> questOdometryPose
    ) {
        Transform2d wheelTransform = wheelOdometryPose.getFirst().minus(lastWheelOdomPose);
        Transform2d questTransform = questOdometryPose.getFirst().minus(lastQuestOdomPose);

        update(
            new Pair<Transform2d, Matrix<N3, N1>>(wheelTransform, wheelOdometryPose.getSecond()),
            new Pair<Transform2d,Matrix<N3,N1>>(questTransform, questOdometryPose.getSecond()),
            visionMeasurementArray
        );

        lastWheelOdomPose = wheelOdometryPose.getFirst();
        lastQuestOdomPose = questOdometryPose.getFirst();
    }

    public void update(
        Pair<Transform2d, Matrix<N3, N1>> deltaWheelOdometry, 
        Pair<Transform2d, Matrix<N3, N1>> deltaQuestOdometry, 
        Pair<Pose2d, Matrix<N3, N1>>[] visionMeasurementArray
    ) {
        Time newPoseTimestamp = Seconds.of(Timer.getFPGATimestamp());
        Time deltaTimestamp = newPoseTimestamp.minus(lastPoseTimestamp);

        Gaussian wheelOdomX = new Gaussian(deltaWheelOdometry.getFirst().getX(), deltaWheelOdometry.getSecond().getData()[0]);
        Gaussian wheelOdomY = new Gaussian(deltaWheelOdometry.getFirst().getY(), deltaWheelOdometry.getSecond().getData()[1]);
        Gaussian wheelOdomTheta = new Gaussian(deltaWheelOdometry.getFirst().getRotation().getRadians(), deltaWheelOdometry.getSecond().getData()[2]);

        Gaussian questOdomX = new Gaussian(deltaWheelOdometry.getFirst().getX(), deltaWheelOdometry.getSecond().getData()[0]);
        Gaussian questOdomY = new Gaussian(deltaWheelOdometry.getFirst().getY(), deltaWheelOdometry.getSecond().getData()[1]);
        Gaussian questOdomTheta = new Gaussian(deltaWheelOdometry.getFirst().getRotation().getRadians(), deltaWheelOdometry.getSecond().getData()[2]);

        List<Gaussian> visionX = new ArrayList<>();
        List<Gaussian> visionY = new ArrayList<>();
        List<Gaussian> visionTheta = new ArrayList<>();

        for (Pair<Pose2d, Matrix<N3, N1>> measurement : visionMeasurementArray) {
            visionX.add(new Gaussian(measurement.getFirst().getX(), measurement.getSecond().getData()[0]));
            visionY.add(new Gaussian(measurement.getFirst().getY(), measurement.getSecond().getData()[1]));
            visionTheta.add(new Gaussian(measurement.getFirst().getRotation().getRadians(), measurement.getSecond().getData()[2]));
        }

        Gaussian combinedOdomX = combineGaussians(wheelOdomX, questOdomX);
        Gaussian combinedOdomY = combineGaussians(wheelOdomY, questOdomY);
        Gaussian combinedOdomTheta = combineGaussians(wheelOdomTheta, questOdomTheta);

        Gaussian odomPoseX = new Gaussian(combinedOdomX.getMean() + currentPose.getX(), combinedOdomX.getStd());
        Gaussian odomPoseY = new Gaussian(combinedOdomY.getMean() + currentPose.getY(), combinedOdomY.getStd());
        Gaussian odomPoseTheta = new Gaussian(combinedOdomTheta.getMean() + currentPose.getRotation().getRadians(), combinedOdomTheta.getStd());

        Gaussian combinedVisionX = combineGaussians(visionX.toArray(new Gaussian[0]));
        Gaussian combinedVisionY = combineGaussians(visionY.toArray(new Gaussian[0]));
        Gaussian combinedVisionTheta = combineGaussians(visionTheta.toArray(new Gaussian[0]));

        


        lastPoseTimestamp = newPoseTimestamp;
    }

    private Gaussian combineGaussians(Gaussian... gaussians) {
        double totalPrecision = 0; // note that precision is the inverse of the standard deviation.
        double totalWeightedMean = 0; // note that this is the mean times it's precision.
        for (Gaussian g : gaussians) {

            if (g.getStd() == 0) { // avoid / 0 error. std = 0 means 100% certain.
                return new Gaussian(g.getMean(), g.getStd());
            }

            double precision = 1.0 / g.getStd();
            totalPrecision += precision;
            
            double weightedMean = precision * g.getMean();
            totalWeightedMean += weightedMean;
        }

        if (totalPrecision == 0) { // should be impossible, but prevent / 0 error in any case.
            return null;
        }

        double newSTD = 1.0 / totalPrecision;
        double newMean = totalWeightedMean / totalPrecision;

        return new Gaussian(newMean, newSTD);
    }

    private class Gaussian {
    
        @Getter @Setter private double mean;
        @Getter @Setter private double std;

        public Gaussian(double mean, double std) {
            this.mean = mean;
            this.std = std;
        }
        
    }
    
}
