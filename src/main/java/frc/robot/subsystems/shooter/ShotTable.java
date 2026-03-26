package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import frc.lib.frc1731.math.regression.*;

public class ShotTable {
    private Regression hoodModel;
    private Regression flywheelModel;
    private Regression tofModel;

    private static List<ShotEntry> kScoreEntries = List.of(
        new ShotEntry(0, 0, 28, 0.5),
        new ShotEntry(1, 0, 28, 0.5),
        new ShotEntry(2, 0.5, 35, 0.75),
        new ShotEntry(3, 1, 42.5, 0.85),
        new ShotEntry(4, 1.5, 47.5, 1.25),
        new ShotEntry(5, 2, 52.5, 1.4),
        new ShotEntry(6, 3, 55, 1.6),
        new ShotEntry(7, 4, 60, 1.8),
        new ShotEntry(8, 4.5, 60, 2.0)
    );

    private static List<ShotEntry> kPassEntries = List.of(
        new ShotEntry(0, 0, 28, 0.5),
        new ShotEntry(1, 0, 28, 0.5),
        new ShotEntry(2, 0.5, 35, 0.75),
        new ShotEntry(3, 1, 42.5, 0.85),
        new ShotEntry(4, 1.5, 47.5, 1.25),
        new ShotEntry(5, 2, 52.5, 1.4),
        new ShotEntry(6, 3, 55, 1.6),
        new ShotEntry(7, 4, 60, 1.8),
        new ShotEntry(8, 4.5, 60, 2.0)
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
        return new ShotTable(kScoreEntries);
    }

    public static ShotTable getPassTable() {
        return new ShotTable(kPassEntries);
    }

    public double[] getShotParameters(double distance) {
        return new double[] {hoodModel.getInterpolation(distance), flywheelModel.getInterpolation(distance), tofModel.getInterpolation(distance)};
    }
}