package frc.robot.subsystems.power;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import frc.lib.frc1731.subsystem.BaseSubsystem;

public class PowerSubsystem extends BaseSubsystem {
    private PowerDistribution pdp;
    private PDPSim pdpSim;

    public PowerSubsystem() {
        // pdp = LoggedPowerDistribution.getInstance();
        pdp = new PowerDistribution();
        pdpSim = new PDPSim();
        RobotController.getBatteryVoltage();
        pdpSim.setInitialized(true);
    }

    @Override
    public void periodicTelemetry() {
        pdpSim.setVoltage(pdp.getVoltage());
        pdpSim.setTemperature(pdp.getTemperature());
        double[] currents = pdp.getAllCurrents();
        for (int i = 0; i < pdp.getAllCurrents().length; i++) {
            pdpSim.setCurrent(i, currents[i]);
        }

        logger.log("Battery Voltage", RobotController.getBatteryVoltage());
        logger.log("CAN Buses Off", RobotController.getCANStatus().busOffCount);
    }
}