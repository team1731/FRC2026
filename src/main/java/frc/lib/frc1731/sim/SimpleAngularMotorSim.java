package frc.lib.frc1731.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.frc1678.sim.PivotSim;
import frc.lib.frc1678.sim.PivotSim.PivotSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.subsystem.converter.AngularSubsystemConverter;

public class SimpleAngularMotorSim {
    private PivotSim sim;
    private PIDController simPIDCtrl;
    private ArmFeedforward simFF;
    private Voltage appliedVoltage = Volts.zero();
    private AngularVelocity lastVelocity = RotationsPerSecond.zero();
    private AngularSubsystemConverter converter;
    private ProfiledPIDController profiledPID;

    public SimpleAngularMotorSim(PivotSimConstants constants, PIDGains gains) {
        sim = new PivotSim(constants);
        simPIDCtrl = gains.toPIDController();
        simFF = new ArmFeedforward(gains.kS, gains.kG, gains.kV, gains.kA);
        converter = new AngularSubsystemConverter(constants.gearing);
        profiledPID = new ProfiledPIDController(gains.kP, gains.kI, gains.kD, new Constraints(12, Double.POSITIVE_INFINITY));
    }

    public void setVoltage(Voltage volts) {
        appliedVoltage = volts;
        sim.setVoltage(appliedVoltage);
    }

    public void setMechanismAngle(Angle angle) {
        Angle motorAngle = converter.toMotor(getMechanismAngle());
        Angle targetMotorAngle = converter.toMotor(angle);
        double pidOutput = profiledPID.calculate(motorAngle.in(Rotations), targetMotorAngle.in(Rotations));
        double ffOutput = simFF.calculateWithVelocities(angle.in(Radians), getMotorVelocity(lastVelocity).in(RadiansPerSecond), getMotorVelocity(getMechanismVelocity()).in(RadiansPerSecond));
        appliedVoltage = Volts.of(pidOutput + ffOutput);
        SmartDashboard.putNumber("1234", getMotorVelocity(lastVelocity).in(RadiansPerSecond));
        SmartDashboard.putNumber("5678", getMotorVelocity(getMechanismVelocity()).in(RadiansPerSecond));
        SmartDashboard.putNumber("1122122ASJKLDKLSJADJKLASJLK", pidOutput);
        SmartDashboard.putNumber("2223223ASJKLDKLSJADJKLASJLK", ffOutput);
        sim.setVoltage(appliedVoltage);
    }

    public Angle getMechanismAngle() {
        return sim.getPosition();
    }

    public AngularVelocity getMechanismVelocity() {
        return sim.getVelocity();
    }

    public void periodic() {
        sim.simulate();
        lastVelocity = getMechanismVelocity();
    }

    public Voltage getAppliedVoltage() {
        return appliedVoltage;
    }

    private AngularVelocity getMotorVelocity(AngularVelocity mechVelocity) {
        return converter.toMotor(mechVelocity.times(Seconds.of(1))).per(Seconds);
    }
}
