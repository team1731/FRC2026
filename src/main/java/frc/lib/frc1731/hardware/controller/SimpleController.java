package frc.lib.frc1731.hardware.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class SimpleController {
    private int port;

    public SimpleController(int port) {
        this.port = port;
    }

    public abstract Trigger a();
    public abstract Trigger b();
    public abstract Trigger x();
    public abstract Trigger y();

    public abstract Trigger leftBumper();
    public abstract Trigger rightBumper();
    public abstract Trigger leftTrigger();
    public abstract Trigger rightTrigger();

    public abstract Trigger leftOptions();
    public abstract Trigger rightOptions();

    public abstract Trigger leftStick();
    public abstract Trigger rightStick();

    public abstract Trigger dpadUp();
    public abstract Trigger dpadDown();
    public abstract Trigger dpadLeft();
    public abstract Trigger dpadRight();

    public abstract double getLeftX();
    public abstract double getLeftY();
    public abstract double getRightX();
    public abstract double getRightY();

    public abstract boolean isConnected();

    public int getPort() {
        return port;
    }
}