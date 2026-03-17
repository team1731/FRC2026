package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import frc.lib.frc1731.math.regression.*;

public class ShotTable {
    private Regression hoodModel;
    private Regression flywheelModel;
    private Regression tofModel;

    private List<ShotEntry> entries = List.of(
        new ShotEntry(Units.inchesToMeters(45.5), 0, 28, 0.56),
        new ShotEntry(Units.inchesToMeters(72.5), 0.5, 30, 0.71),
        new ShotEntry(Units.inchesToMeters(91), 1, 38, 0.89),
        new ShotEntry(Units.inchesToMeters(128), 2, 43, 1.26),
        new ShotEntry(Units.inchesToMeters(150), 2.5, 50, 1.45),
        new ShotEntry(Units.inchesToMeters(180), 3.5, 58, 1.6)
    );

    private List<ShotEntry> interpolatingEntries = List.of(
        new ShotEntry(0, 0, 28, 0.5),
        new ShotEntry(1, 0, 28, 0.5),
        new ShotEntry(2, 0.5, 35, 0.75),
        new ShotEntry(3, 1, 45, 0.85),
        new ShotEntry(4, 1, 50, 1.25),
        new ShotEntry(5, 1.5, 60, 1.4),
        new ShotEntry(6, 3, 60, 1.6),
        new ShotEntry(7, 4, 60, 1.8),
        new ShotEntry(8, 5, 60, 2.0)
    );

    public ShotTable() {
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

        hoodModel = new LinearRegression(distances.toArray(new Double[0]), hoodAngles.toArray(new Double[0]));
        flywheelModel = new LinearRegression(distances.toArray(new Double[0]), flywheelSpeeds.toArray(new Double[0]));
        tofModel = new LinearRegression(distances.toArray(new Double[0]), tofs.toArray(new Double[0]));
    }

    public ShotTable backup() {
        List<Double> distances = new ArrayList<>();
        List<Double> hoodAngles = new ArrayList<>();
        List<Double> flywheelSpeeds = new ArrayList<>();
        List<Double> tofs = new ArrayList<>();

        for (ShotEntry entry : interpolatingEntries) {
            distances.add(entry.distance);
            hoodAngles.add(entry.hoodRotations);
            flywheelSpeeds.add(entry.flywheelRPS);
            tofs.add(entry.timeOfFlight);
        }

        hoodModel = new PiecewiseRegression(hoodAngles.toArray(new Double[0]), 1d);
        flywheelModel = new PiecewiseRegression(flywheelSpeeds.toArray(new Double[0]), 1d);
        tofModel = new PiecewiseRegression(tofs.toArray(new Double[0]), 1d);
        return this;
    }

    public double[] getShotParameters(double distance) {
        return new double[] {hoodModel.getInterpolation(distance), flywheelModel.getInterpolation(distance), tofModel.getInterpolation(distance)};
    }
}