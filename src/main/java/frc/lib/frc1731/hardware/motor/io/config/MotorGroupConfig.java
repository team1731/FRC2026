package frc.lib.frc1731.hardware.motor.io.config;

import java.util.List;
import java.util.Objects;

/** Construction snapshot for one leader and same-family followers on its CAN bus. */
public record MotorGroupConfig<C>(C leader, List<Follower<C>> followers) {
    public MotorGroupConfig {
        Objects.requireNonNull(leader);
        followers = List.copyOf(followers);
    }

    /** Inversion is relative to the leader output, not the follower encoder. */
    public record Follower<C>(int deviceId, boolean inverted, C config) {
        public Follower {
            requireDeviceId(deviceId);
            Objects.requireNonNull(config);
        }
    }

    public static void requireDeviceId(int id) {
        if (id < 0 || id > 62) throw new IllegalArgumentException("CAN ID must be between 0 and 62");
    }
}
