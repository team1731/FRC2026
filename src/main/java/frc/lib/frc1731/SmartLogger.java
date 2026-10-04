package frc.lib.frc1731;

import java.util.function.BooleanSupplier;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.util.struct.StructSerializable;

/**
 * Small wrapper for consistently logging team subsystem data into AdvantageKit.
 *
 * <p>Each instance writes under {@code /SmartLogs/<name>/} and can be globally enabled or disabled
 * through a supplied condition for derived outputs. Inputs are always processed for capture/replay.
 * Use stable keys and call from the robot main thread.
 */
public class SmartLogger {
    private final String m_folder;
    private final String m_inputsPath;
    private final BooleanSupplier m_shouldLog;
    private final Map<String, String> m_outputPaths = new HashMap<>();

    /**
     * Creates a logger rooted under a named SmartLogs folder.
     *
     * @param name folder name to use for all values written by this logger
     * @param shouldLog condition that controls whether values are emitted
     */
    public SmartLogger(String name, BooleanSupplier shouldLog) {
        this.m_folder = "/SmartLogs/" + name + "/";
        this.m_inputsPath = "RealOutputs/" + m_folder;
        this.m_shouldLog = shouldLog;
    }

    /** Whether optional derived outputs are enabled; does not control input capture. */
    public boolean isOutputEnabled() { return m_shouldLog.getAsBoolean(); }

    // Keys must be stable field names, not timestamps or changing values. Main-thread use only.
    private String outputPath(String key) {
        String path = m_outputPaths.get(key);
        if (path == null) {
            path = m_folder + key;
            m_outputPaths.put(key, path);
        }
        return path;
    }

    /**
     * Logs a double to AK
     *
     * @param key value name relative to this logger's folder
     * @param value value to record
     */
    public void log(String key, double value) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), value);
    }

    /**
     * Logs a boolean to AK
     *
     * @param key value name relative to this logger's folder
     * @param value value to record
     */
    public void log(String key, boolean value) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), value);
    }

    /**
     * Logs a WPILib geometry class (Pose2d, ChassisSpeeds, etc.) to AK
     *
     * @param key value name relative to this logger's folder
     * @param value struct-serializable value to record
     */
    public void log(String key, StructSerializable value) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), value);
    }

    /**
     * Logs a String to AK
     *
     * @param key value name relative to this logger's folder
     * @param value value to record
     */
    public void log(String key, String value) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), value);
    }

    /**
     * Logs an enum value to AK
     *
     * @param key value name relative to this logger's folder
     * @param value enum value to record
     * @param <T> enum type being logged
     */
    public <T extends Enum<T>> void log(String key, T value) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), value);
    }

    /**
     * Logs a double to AK if the indicated condition is true, otherwise logs a different value
     *
     * @param key value name relative to this logger's folder
     * @param valueIfTrue value recorded when condition is true
     * @param valueIfFalse value recorded when condition is false
     * @param condition branch condition
     */
    public void logIf(String key, double valueIfTrue, double valueIfFalse, boolean condition) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), condition ? valueIfTrue : valueIfFalse); // Record to AdvantageKit logs
    }

    /**
     * Logs a boolean to AK if the indicated condition is true, otherwise logs a different value
     *
     * @param key value name relative to this logger's folder
     * @param valueIfTrue value recorded when condition is true
     * @param valueIfFalse value recorded when condition is false
     * @param condition branch condition
     */
    public void logIf(String key, boolean valueIfTrue, boolean valueIfFalse, boolean condition) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), condition ? valueIfTrue : valueIfFalse); // Record to AdvantageKit logs
    }

    /**
     * Logs a String to AK if the indicated condition is true, otherwise logs a different value
     *
     * @param key value name relative to this logger's folder
     * @param valueIfTrue value recorded when condition is true
     * @param valueIfFalse value recorded when condition is false
     * @param condition branch condition
     */
    public void logIf(String key, String valueIfTrue, String valueIfFalse, boolean condition) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), condition ? valueIfTrue : valueIfFalse); // Record to AdvantageKit logs
    }

    /**
     * Logs a WPILib geometry class (Pose2d, ChassisSpeeds, etc.) to AK if the indicated condition is true, otherwise logs a different value
     *
     * @param key value name relative to this logger's folder
     * @param valueIfTrue value recorded when condition is true
     * @param valueIfFalse value recorded when condition is false
     * @param condition branch condition
     */
    public void logIf(String key, StructSerializable valueIfTrue, StructSerializable valueIfFalse, boolean condition) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), condition ? valueIfTrue : valueIfFalse); // Record to AdvantageKit logs
    }

    /**
     * Logs an enum value to AK if the indicated condition is true, otherwise logs a different value
     *
     * @param key value name relative to this logger's folder
     * @param valueIfTrue value recorded when condition is true
     * @param valueIfFalse value recorded when condition is false
     * @param condition branch condition
     * @param <E> enum type being logged
     */
    public <E extends Enum<E>> void logIf(String key, E valueIfTrue, E valueIfFalse, boolean condition) {
      if (!m_shouldLog.getAsBoolean()) return;
      Logger.recordOutput(outputPath(key), condition ? valueIfTrue : valueIfFalse); // Record to AdvantageKit logs
    }

    /**
     * Processes inputs every loop, independently of optional output logging.
     * Preserves the existing input path for log compatibility; replay may populate these inputs.
     * @param inputs {@code AutoLogged} inputs object with all information needed for logging
     */
    public void processInputs(LoggableInputs inputs) {
      Logger.processInputs(m_inputsPath, inputs);
    }
}
