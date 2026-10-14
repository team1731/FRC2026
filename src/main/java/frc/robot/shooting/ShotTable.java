package frc.robot.shooting;

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
        new ShotEntry(2, 5, 55, 1.16),
        new ShotEntry(3, 9, 62.5, 1.2),
        new ShotEntry(4, 12, 70.0, 1.2),
        new ShotEntry(5, 16, 75.0,1.25),
        new ShotEntry(6, 17, 82.5, 2.1),
        new ShotEntry(7, 18, 90, 2.4),
        new ShotEntry(8, 20, 95, 2.8)
    );

    private static final List<ShotEntry> kPassEntries = List.of(
        new ShotEntry(0, 18, 40, 0.5),
        new ShotEntry(1, 18, 45, 0.5),
        new ShotEntry(2, 18, 45, 0.75),
        new ShotEntry(3, 18, 45, 0.85),
        new ShotEntry(4, 18, 55, 1.25),
        new ShotEntry(5, 18, 55, 1.4),
        new ShotEntry(6, 18, 65, 1.6),
        new ShotEntry(7, 18, 65, 1.8),
        new ShotEntry(8, 18, 85, 2.0),
        new ShotEntry(9, 18, 90, 2.0),
        new ShotEntry(10, 18, 90, 2.0),
        new ShotEntry(11, 18, 92.5, 2.0),
        new ShotEntry(12, 18, 95, 2.0)
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