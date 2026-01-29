package frc.lib.frc1731.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1678.sim.RollerSim;
import frc.lib.frc1678.sim.RollerSim.RollerSimConstants;
import frc.lib.frc1731.PIDGains;

public class SimpleVelocitySim {
    private RollerSim sim;
    private PIDController simPIDCtrl;
    private SimpleMotorFeedforward simFF;
    private double gearing = 1d;
    private Voltage appliedVoltage = Volts.zero();

    public SimpleVelocitySim(DCMotor motor, double gearing, Distance radius, Mass weight, PIDGains gains) {
        sim = new RollerSim(new RollerSimConstants(motor, gearing, 0.5 * weight.in(Kilograms) * Math.pow(radius.in(Meters), 2)));
        this.gearing = gearing;
        this.simPIDCtrl = gains.toPIDController();
        this.simFF = new SimpleMotorFeedforward(gains.kS, gains.kV, gains.kA);
    }

    public static SimpleVelocitySim buildKrakenX60Sim(int numMotors, double gearing, Distance radius, Mass weight, PIDGains gains) {
        return new SimpleVelocitySim(DCMotor.getKrakenX60(numMotors), gearing, radius, weight, gains);
    }

    public static SimpleVelocitySim buildKrakenX44Sim(int numMotors, double gearing, Distance radius, Mass weight, PIDGains gains) {
        return new SimpleVelocitySim(DCMotor.getKrakenX44(numMotors), gearing, radius, weight, gains);
    }

    public static SimpleVelocitySim buildCIMSim(int numMotors, double gearing, Distance radius, Mass weight, PIDGains gains) {
        return new SimpleVelocitySim(DCMotor.getCIM(numMotors), gearing, radius, weight, gains);
    }

    public void setVelocity(AngularVelocity velocity) {
        double pidOutput = simPIDCtrl.calculate(gearing);
        double ffOutput = simFF.calculate(velocity.in(RotationsPerSecond));
        appliedVoltage = Volts.of(pidOutput + ffOutput);
        sim.setVoltage(appliedVoltage);
    }

    public void setVoltage(Voltage volts) {
        appliedVoltage = volts;
        sim.setVoltage(volts);
    }

    public AngularVelocity getVelocity() {
        return sim.getVelocity();
    }

    public Angle getPosition() {
        return sim.getPosition();
    }

    public Voltage getAppliedVoltage() {
        return appliedVoltage;
    }

    public void periodic() {
        sim.simulate();
    }
}