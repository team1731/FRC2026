package frc.lib.frc1731.hardware.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimpleXboxController extends SimpleController {
    private CommandXboxController ctrl;
    public SimpleXboxController(int port) {
        super(port);
        ctrl = new CommandXboxController(port);
    }

    @Override
    public Trigger a() {
        return ctrl.a();
    }

    @Override
    public Trigger b() {
        return ctrl.b();
    }

    @Override
    public Trigger x() {
        return ctrl.x();
    }

    @Override
    public Trigger y() {
        return ctrl.y();
    }

    @Override
    public Trigger leftBumper() {
        return ctrl.leftBumper();
    }

    @Override
    public Trigger rightBumper() {
        return ctrl.rightBumper();
    }

    @Override
    public Trigger leftTrigger() {
        return ctrl.leftTrigger();
    }

    @Override
    public Trigger rightTrigger() {
        return ctrl.rightTrigger();
    }

    @Override
    public Trigger leftOptions() {
        return ctrl.back();
    }

    @Override
    public Trigger rightOptions() {
        return ctrl.start();
    }

    @Override
    public Trigger leftStick() {
        return ctrl.leftStick();
    }

    @Override
    public Trigger rightStick() {
        return ctrl.rightStick();
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