package frc.lib.frc1731.log;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.util.struct.StructSerializable;

/**
 * Class to easily log to AdvantageKit under a named folder
 */
public class SmartLogger {
    private String logFolder = "/SmartLogs/";
    private BooleanSupplier shouldLog;

    public SmartLogger(String name, BooleanSupplier shouldLog) {
        this.logFolder = logFolder + name + "/";
        this.shouldLog = shouldLog;
    }

    /**
     * Logs a double to AK
     */
    public void log(String key, double value) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs a boolean to AK
     */
    public void log(String key, boolean value) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs a WPILib geometry class (Pose2d, ChassisSpeeds, etc.) to AK
     */
    public void log(String key, StructSerializable value) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs a String to AK
     */
    public void log(String key, String value) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs an enum value to AK
     */
    public <T extends Enum<T>> void log(String key, T value) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, value);
    }

    /**
     * Logs a double to AK if the indicated condition is true, otherwise logs a different value
     */
    public void logIf(String key, double valueIfTrue, double valueIfFalse, boolean condition) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, condition ? valueIfTrue : valueIfTrue); // Record to AdvantageKit logs
    }

    /**
     * Logs a boolean to AK if the indicated condition is true, otherwise logs a different value
     */
    public void logIf(String key, boolean valueIfTrue, boolean valueIfFalse, boolean condition) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, condition ? valueIfTrue : valueIfTrue); // Record to AdvantageKit logs
    }

    /**
     * Logs a String to AK if the indicated condition is true, otherwise logs a different value
     */
    public void logIf(String key, String valueIfTrue, String valueIfFalse, boolean condition) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, condition ? valueIfTrue : valueIfTrue); // Record to AdvantageKit logs
    }

    /**
     * Logs a WPILib geometry class (Pose2d, ChassisSpeeds, etc.) to AK if the indicated condition is true, otherwise logs a different value
     */
    public void logIf(String key, StructSerializable valueIfTrue, StructSerializable valueIfFalse, boolean condition) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, condition ? valueIfTrue : valueIfTrue); // Record to AdvantageKit logs
    }

    /**
     * Logs an enum value to AK if the indicated condition is true, otherwise logs a different value
     */
    public <E extends Enum<E>> void logIf(String key, E valueIfTrue, E valueIfFalse, boolean condition) {
        if (!shouldLog.getAsBoolean()) return;
       Logger.recordOutput(logFolder + key, condition ? valueIfTrue : valueIfTrue); // Record to AdvantageKit logs
    }

    public void processInputs(LoggableInputs inputs) {
      Logger.processInputs("RealOutputs/" + logFolder, inputs);
    }
}