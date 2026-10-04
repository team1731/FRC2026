package frc.lib.frc1731.hardware.motor.io.request;

import frc.lib.frc1731.hardware.motor.io.MotorIO;

/** Requests zero output with coast behavior. */
public class CoastRequest extends MotorRequest {
    public CoastRequest() {
        super(RequestType.kIdle);
    }

    @Override
    public CoastRequest withSlot(int slot) {
        super.withSlot(slot); return this;
    }

    @Override
    public double getBaseValue() {
        return 0.0;
    }

    @Override
    public void apply(MotorIO io) {
        io.coast();
    }
}
