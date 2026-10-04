package frc.lib.frc1731;

import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.UUID;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Routes USB logs by FMS match, retaining ordinary robot operation in testing. */
public final class EventLogWriter implements LogDataReceiver {
  private static final DateTimeFormatter TIME =
      DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");
  private final Path root;
  private final String eventKey;
  private WPILOGWriter writer;
  private String currentMatch = "";
  private boolean wasEnabled;

  /** An empty event key uses the event name supplied by FMS. */
  public EventLogWriter(String root, String eventKey) {
    this.root = Path.of(root);
    this.eventKey = sanitize(eventKey);
  }

  @Override
  public void start() {
    open(Path.of("testing"), "testing");
  }

  @Override
  public void putTable(LogTable table) {
    boolean enabled = table.get("DriverStation/Enabled", false);
    boolean fms = table.get("DriverStation/FMSAttached", false);
    int type = table.get("DriverStation/MatchType", 0);
    int number = table.get("DriverStation/MatchNumber", 0);
    int replay = table.get("DriverStation/ReplayNumber", 0);
    String event = eventKey.isEmpty()
        ? sanitize(table.get("DriverStation/EventName", "")) : eventKey;
    if (event.isEmpty()) event = "unknown-event";

    String category = switch (type) {
      case 1 -> "practice";
      case 2 -> "quals";
      case 3 -> "playoffs";
      default -> "";
    };
    if (fms && !category.isEmpty() && number > 0) {
      String match = category + String.format("_%03d_r%d", number, replay);
      String identity = event + "/" + match;
      if (!identity.equals(currentMatch)) {
        open(Path.of("competition", event, category), match);
        currentMatch = identity;
      }
    } else if (enabled && !wasEnabled && !fms && !currentMatch.isEmpty()) {
      // Keep post-match disabled data with the match, until a local session starts.
      open(Path.of("testing"), "testing");
      currentMatch = "";
    }
    wasEnabled = enabled;
    // A fresh WPILOGWriter writes every field of this complete table after rotation.
    writer.putTable(table);
  }

  private void open(Path folder, String label) {
    if (writer != null) writer.end();
    String filename = label + "_" + TIME.format(LocalDateTime.now())
        + "_" + UUID.randomUUID() + ".wpilog";
    writer = new WPILOGWriter(root.resolve(folder).resolve(filename).toString());
    writer.start();
  }

  @Override
  public void end() {
    if (writer != null) writer.end();
  }

  private static String sanitize(String value) {
    return value.trim().toLowerCase(java.util.Locale.ROOT)
        .replaceAll("[^a-z0-9_-]+", "_");
  }
}
