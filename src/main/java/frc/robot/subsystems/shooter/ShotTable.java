package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import frc.lib.frc1731.math.regression.*;

public class ShotTable {
    private Regression hoodModel;
    private Regression flywheelModel;
    private Regression tofModel;

    private static final List<ShotEntry> kHubEntries = List.of(
        new ShotEntry(0, 0.0, 55, 0.5),
        new ShotEntry(1, 0.0, 55, 0.5),
        new ShotEntry(2, 5, 55, 1.25),
        new ShotEntry(3, 6, 62.5, 1.4),
        new ShotEntry(4, 9, 70, 1.8),
        new ShotEntry(5, 15, 80,2.25),
        new ShotEntry(6, 17, 87.5, 2.5),
        new ShotEntry(7, 18, 92.5, 2.8),
        new ShotEntry(8, 20, 94.5, 3.0)
    );

    private static final List<ShotEntry> kPassEntries = List.of(
        new ShotEntry(0, 0, 40, 0.5),
        new ShotEntry(1, 5, 50, 0.5),
        new ShotEntry(2, 6, 50, 0.75),
        new ShotEntry(3, 7, 60, 0.85),
        new ShotEntry(4, 10, 60, 1.25),
        new ShotEntry(5, 15, 70, 1.4),
        new ShotEntry(6, 15, 70, 1.6),
        new ShotEntry(7, 15, 80, 1.8),
        new ShotEntry(8, 15, 80, 2.0),
        new ShotEntry(9, 15, 85, 2.0),
        new ShotEntry(10, 20, 85, 2.0),
        new ShotEntry(11, 20, 95, 2.0),
        new ShotEntry(12, 20, 95, 2.0),
        new ShotEntry(13, 20, 95, 2.0),
        new ShotEntry(14, 7, 95, 2.0),
        new ShotEntry(15, 7, 95, 2.0),
        new ShotEntry(16, 7, 95, 2.0),
        new ShotEntry(17, 7, 95, 2.0)
    );

    private ShotTable(List<ShotEntry> entries) {
        List<Double> distances = new ArrayList<>();
        List<Double> hoodAngles = new ArrayList<>();
        List<Double> flywheelSpeeds = new ArrayList<>();
        List<Double> tofs = new ArrayList<>();

        for (ShotEntry entry : entries) {
            distances.add(entry.distance);
            hoodAngles.add(entry.hoodRotations);
            flywheelSpeeds.add(entry.flywheelRPS);
            tofs.add(entry.timeOfFlight);
        }

        hoodModel = new PiecewiseRegression(hoodAngles.toArray(new Double[0]), 1d);
        flywheelModel = new PiecewiseRegression(flywheelSpeeds.toArray(new Double[0]), 1d);
        tofModel = new PiecewiseRegression(tofs.toArray(new Double[0]), 1d);
    }

    public static ShotTable getScoringTable() {
        return new ShotTable(kHubEntries);
    }

    public static ShotTable getPassingTable() {
        return new ShotTable(kPassEntries);
    }

    public double[] getShotParameters(double distance) {
        return new double[] {hoodModel.getInterpolation(distance), flywheelModel.getInterpolation(distance), tofModel.getInterpolation(distance)};
    }
}