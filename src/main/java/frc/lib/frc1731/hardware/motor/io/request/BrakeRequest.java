package frc.lib.frc1731.hardware.motor.io.request;

import frc.lib.frc1731.hardware.motor.io.MotorIO;

/** Requests zero output with brake behavior. */
public class BrakeRequest extends MotorRequest {
    public BrakeRequest() {
        super(RequestType.kIdle);
    }

    @Override
    public BrakeRequest withSlot(int slot) {
        super.withSlot(slot); return this;
    }

    @Override
    public double getBaseValue() {
        return 0.0;
    }

    @Override
    public void apply(MotorIO io) {
        io.brake();
    }
}
