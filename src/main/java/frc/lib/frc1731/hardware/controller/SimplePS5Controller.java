package frc.lib.frc1731.hardware.controller;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimplePS5Controller extends SimpleController {
    private CommandPS5Controller ctrl;
    public SimplePS5Controller(int port) {
        super(port);
        ctrl = new CommandPS5Controller(port);
    }

    @Override
    public Trigger a() {
        return ctrl.cross();
    }

    @Override
    public Trigger b() {
        return ctrl.circle();
    }

    @Override
    public Trigger x() {
        return ctrl.square();
    }

    @Override
    public Trigger y() {
        return ctrl.triangle();
    }

    @Override
    public Trigger leftBumper() {
        return ctrl.L1();
    }

    @Override
    public Trigger rightBumper() {
        return ctrl.R1();
    }

    @Override
    public Trigger leftTrigger() {
        return ctrl.L2();
    }

    @Override
    public Trigger rightTrigger() {
        return ctrl.R2();
    }

    @Override
    public Trigger leftOptions() {
        return ctrl.create();
    }

    @Override
    public Trigger rightOptions() {
        return ctrl.options();
    }

    @Override
    public Trigger leftStick() {
        return new Trigger(() -> false); // PS5 controller does not have a left stick button
    }

    @Override
    public Trigger rightStick() {
        return new Trigger(() -> false); // PS5 controller does not have a right stick button
    }

    @Override
    public Trigger dpadUp() {
        return ctrl.povUp();
    }

    @Override
    public Trigger dpadDown() {
        return ctrl.povDown();
    }

    @Override
    public Trigger dpadLeft() {
        return ctrl.povLeft();
    }

    @Override
    public Trigger dpadRight() {
        return ctrl.povRight();
    }

    @Override
    public double getLeftX() {
        return ctrl.getLeftX();
    }

    @Override
    public double getLeftY() {
        return ctrl.getLeftY();
    }

    @Override
    public double getRightX() {
        return ctrl.getRightX();
    }

    @Override
    public double getRightY() {
        return ctrl.getRightY();
    }

    @Override
    public boolean isConnected() {
        return ctrl.isConnected();
    }
}