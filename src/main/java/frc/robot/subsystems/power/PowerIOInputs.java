package frc.robot.subsystems.power;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class PowerIOInputs {
    public double totalCurrentDraw, batteryVoltage, totalPowerDraw;
    public boolean isBrowningOut;
    public int busOffCount;
}