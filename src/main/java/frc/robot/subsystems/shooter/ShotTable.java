package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import frc.lib.frc1731.math.regression.*;

public class ShotTable {
    private Regression hoodModel;
    private Regression flywheelModel;
    private Regression tofModel;

    private static final List<ShotEntry> kHubEntries = List.of(
        new ShotEntry(0, 0.0, 28.0, 0.5),
        new ShotEntry(1, 0.0, 28.0, 0.5),
        new ShotEntry(2, 0.5, 35.0, 1.25),
        new ShotEntry(3, 1.5, 45, 1.4),
        new ShotEntry(4, 2.5, 50, 1.8),
        new ShotEntry(5, 4.0, 55,1.9),
        new ShotEntry(6, 4.5, 60, 2.0),
        new ShotEntry(7, 5.5, 65, 2.1),
        new ShotEntry(8, 6.5, 70, 2.2)
    );

    private static final List<ShotEntry> kPassEntries = List.of(
        new ShotEntry(0, 0, 15, 0.5),
        new ShotEntry(1, 1, 15, 0.5),
        new ShotEntry(2, 2, 15, 0.75),
        new ShotEntry(3, 3, 20, 0.85),
        new ShotEntry(4, 4, 20, 1.25),
        new ShotEntry(5, 5, 25, 1.4),
        new ShotEntry(6, 6, 30, 1.6),
        new ShotEntry(7, 7, 40, 1.8),
        new ShotEntry(8, 7, 50, 2.0),
        new ShotEntry(9, 7, 55, 2.0),
        new ShotEntry(10, 7, 60, 2.0),
        new ShotEntry(11, 7, 60, 2.0),
        new ShotEntry(12, 7, 60, 2.0),
        new ShotEntry(13, 7, 60, 2.0),
        new ShotEntry(14, 7, 60, 2.0),
        new ShotEntry(15, 7, 60, 2.0),
        new ShotEntry(16, 7, 60, 2.0),
        new ShotEntry(17, 7, 60, 2.0)
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