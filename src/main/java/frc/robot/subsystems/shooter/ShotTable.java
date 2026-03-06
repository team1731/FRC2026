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
        new ShotEntry(Units.inchesToMeters(72.5), 1.5, 30, 0.71),
        new ShotEntry(Units.inchesToMeters(91), 2, 38, 0.89),
        new ShotEntry(Units.inchesToMeters(128), 4, 43, 1.26),
        new ShotEntry(Units.inchesToMeters(150), 5.5, 50, 1.45)
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

    public double[] getShotParameters(double distance) {
        return new double[] {hoodModel.getInterpolation(distance), flywheelModel.getInterpolation(distance), tofModel.getInterpolation(distance)};
    }
}