package frc.lib.frc1731.hardware;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj.util.Color;

public class SimpleCANdle {
    private CANdle candle;
    private CANdleConfiguration config;
    private CANdleConfigurator configurator;
    // private CANdleSimState sim;

    public SimpleCANdle(int deviceID, int stripLength){
        this(deviceID, "rio", stripLength);
    }

    public SimpleCANdle(int deviceID, String canbus, int stripLength){
        this.candle = new CANdle(deviceID, canbus);
        // this.sim = candle.getSimState();

        this.config = new CANdleConfiguration();
        this.config.LED.StripType = StripTypeValue.RGBW;
        this.config.LED.BrightnessScalar = 0.5;
        this.config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        this.configurator = candle.getConfigurator();
        this.configurator.apply(config);

        /* clear all previous animations */
        for (int i = 0; i < 8; ++i) {
            this.candle.setControl(new EmptyAnimation(i));
        }
    }

    public void setColorFlow(Color color, int start, int end) {
        candle.setControl(new ColorFlowAnimation(start, end).withColor(new RGBWColor(color)));
    }

    public void setColorFlow(Color color) {
        this.setColorFlow(color, 0, 7);
    }

    public void setFire(int start, int end) {
        candle.setControl(new FireAnimation(start, end));
    }

    public void setFire() {
        this.setFire(0, 7);
    }

    public void setLarson(Color color, LarsonBounceValue bounceMode, int start, int end) {
        candle.setControl(new LarsonAnimation(start, end).withColor(new RGBWColor(color)).withBounceMode(bounceMode));
    }

    public void setLarson(Color color, LarsonBounceValue bounceMode) {
        this.setLarson(color, bounceMode, 0, 7);
    }

    public void setRainbow(int start, int end) {
        candle.setControl(new RainbowAnimation(start, end));
    }

    public void setRainbow() {
        this.setRainbow(0, 7);
    }

    public void setRGBFade(int start, int end) {
        candle.setControl(new RgbFadeAnimation(start, end));
    }

    public void setRGBFade() {
        this.setRGBFade(0, 7);
    }

    public void setSingleFade(Color color, int start, int end) {
        candle.setControl(new SingleFadeAnimation(start, end).withColor(new RGBWColor(color)));
    }

    public void setSingleFade(Color color) {
        this.setSingleFade(color, 0, 7);
    }

    public void setStrobe(Color color, int start, int end) {
        candle.setControl(new StrobeAnimation(start, end).withColor(new RGBWColor(color)));
    }

    public void setStrobe(Color color) {
        this.setStrobe(color, 0, 7);
    }

    public void setTwinkle(Color color, int start, int end) {
        candle.setControl(new TwinkleAnimation(start, end).withColor(new RGBWColor(color)));
    }

    public void setTwinkle(Color color) {
        this.setTwinkle(color, 0, 7);
    }

    public void setTwinkleOff(Color color, int start, int end) {
        candle.setControl(new TwinkleOffAnimation(start, end).withColor(new RGBWColor(color)));
    }

    public void setTwinkleOff(Color color) {
        this.setTwinkleOff(color, 0, 7);
    }
}