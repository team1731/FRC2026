package frc.robot.subsystems.drive;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;

public class QuestWaker {
    private final String ADB_PATH = "/home/lvuser/adb";
    private final String APP_ID = "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity";
    private final String PORT = "5802";

    private String questIP = "";

    /**
     *  * Scans NetworkTables for a connection that looks like the Quest
     *  
     */
    public void updateQuestIP() {
        ConnectionInfo[] connections = NetworkTableInstance.getDefault().getConnections();
        for (ConnectionInfo conn : connections) {
            // Check for the remote_id used by QuestNav
            if (conn.remote_id.toLowerCase().contains("quest")) {
                // Split to remove the port if present (e.g., 10.17.31.X:58010)
                this.questIP = conn.remote_ip.split(":")[0];
                return;
            }
        }
    }

    /**
     *  * Connects via ADB and starts the QuestNav Activity
     *  
     */
    public void launchQuestNav() {
        if (questIP.isEmpty()) {
            System.out.println("Quest IP not found yet. Cannot launch.");
            return;
        }

        String target = questIP + ":" + PORT;

        // Use a separate thread so we don't lag the main robot loop
        new Thread(() -> {
            try {
                System.out.println("Attempting to launch QuestNav at " + target);

                // 1. ADB Connect
                runCommand(ADB_PATH, "connect", target);

                // 2. ADB Shell Start Activity
                runCommand(ADB_PATH, "-s", target, "shell", "am", "start", "-n", APP_ID);

                System.out.println("Launch commands sent successfully.");
            } catch (Exception e) {
                System.err.println("Failed to launch QuestNav: " + e.getMessage());
            }
        }).start();
    }

    private void runCommand(String... args) throws IOException, InterruptedException {
        ProcessBuilder pb = new ProcessBuilder(args);
        pb.redirectErrorStream(true);
        Process p = pb.start();
        p.waitFor();
    }

    public String getIP() {
        return questIP;
    }
}