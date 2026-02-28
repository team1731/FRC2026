package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import frc.lib.frc1731.math.regression.LinearRegression;
import frc.lib.frc1731.math.regression.Regression;

public class ShotTable {
    private HashMap<Double, Double> kHoodAngleTable = new HashMap<>();
    private HashMap<Double, Double> kFlywheelSpeedTable = new HashMap<>();
    private Regression hoodModel;
    private Regression flywheelModel;

    public ShotTable() {
        List<Double> hoodDistances = List.of(0d, 1d, 2d, 4d);
        List<Double> hoodAngles = List.of(18d, 20d, 25d, 30d);

        ArrayList<Double> flywheelDistances = new ArrayList<>();
        ArrayList<Double> flywheelSpeeds = new ArrayList<>();
        for (double distance : kHoodAngleTable.keySet()) {
            hoodDistances.add(distance);
            hoodAngles.add(kHoodAngleTable.get(distance));
        }

        for (double distance : kFlywheelSpeedTable.keySet()) {
            flywheelDistances.add(distance);
            flywheelSpeeds.add(kFlywheelSpeedTable.get(distance));
        }


        hoodModel = new LinearRegression(hoodDistances.toArray(new Double[0]), hoodAngles.toArray(new Double[0]));
        flywheelModel = new LinearRegression(flywheelDistances.toArray(new Double[0]), flywheelSpeeds.toArray(new Double[0]));
    }

    public double[] getShotParameters(double distance) {
        return new double[] {hoodModel.getInterpolation(distance), flywheelModel.getInterpolation(distance)};
    }
}