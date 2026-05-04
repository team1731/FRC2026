// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.io.File;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


public class QuestNavADBWatcher {

    private static final String QUEST_ADB_ADDRESS = "10.17.31.201:5802"; // IP and port for Quest ADB

    private static final String ADB_PATH = "/home/lvuser/adb"; //where the ADB executable is located on the rio

    private static final double ADB_TRIGGER_DELAY_S = 0.3; //time before trying to fix with ADB

    private static final double ADB_COOLDOWN_S = 2.0; //time between ADB fix attempts

    private static final int ADB_MAX_RETRIES = 5; //number of times to try the ADB fix

    private static final int POLL_INTERVAL_MS = 300; //intervals to check for pause

    private final ScheduledExecutorService executor;

    private double passthroughStartTime = -1.0; 
    private double passthroughLastFixTime = -1.0; 
    private int passthroughRetries = 0;

    public QuestNavADBWatcher() {
        executor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "QuestNavADBWatcher");
            t.setDaemon(true); 
            return t;
        });
    }

    public void start() {
        
        executor.schedule(this::openPort, 0, TimeUnit.SECONDS); //ensure Quest is in TCP mode on 5802 port
        executor.schedule(this::connectADB, 0, TimeUnit.SECONDS); //ensure connection to Quest
        executor.scheduleAtFixedRate(this::poll, 0, POLL_INTERVAL_MS, TimeUnit.MILLISECONDS);
        
    }

    public void stop() {
        executor.shutdown();
    }

    private void openPort() {
        try {
            File adbFile = new File(ADB_PATH);
            if (!adbFile.exists()) {
                System.err.println("[QuestNavADBWatcher] adb not found at: " + ADB_PATH
                        + " — update ADB_PATH in QuestNavADBWatcher.java");
                return;
            }

            Process p = new ProcessBuilder(ADB_PATH, "tcpip", "5802")
                    .redirectErrorStream(true)
                    .start();
            p.waitFor(3, TimeUnit.SECONDS);
            
        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] ADB tcpip command failed: " + e.getMessage());
        }
    }

    private void connectADB() {
        try {
            File adbFile = new File(ADB_PATH);
            if (!adbFile.exists()) {
                System.err.println("[QuestNavADBWatcher] adb not found at: " + ADB_PATH
                        + " — update ADB_PATH in QuestNavADBWatcher.java");
                return;
            }

            Process p = new ProcessBuilder(ADB_PATH, "connect", QUEST_ADB_ADDRESS)
                    .redirectErrorStream(true)
                    .start();
            p.waitFor(3, TimeUnit.SECONDS);
            
        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] ADB connect failed: " + e.getMessage());
        }
    }

    private void poll() {
        try {
           boolean isInPassthrough = true;
            double now = nowSeconds();

            if (isInPassthrough) {
                
                if (passthroughStartTime < 0) {
                    passthroughStartTime = now;
                }

                double elapsedSinceTrigger = now - passthroughStartTime;
                double elapsedSinceLastFix = (passthroughLastFixTime < 0) ? Double.MAX_VALUE : (now - passthroughLastFixTime);

                if (elapsedSinceTrigger > ADB_TRIGGER_DELAY_S) {
                    if (elapsedSinceLastFix > ADB_COOLDOWN_S) {
                        if (passthroughRetries < ADB_MAX_RETRIES) {

                            passthroughRetries++;

                            fireADBRelaunch();

                            passthroughLastFixTime = now;

                            passthroughStartTime = now;

                        }
                    }
                }

            } else {

                if (passthroughStartTime >= 0) {

                    passthroughStartTime = -1.0;

                    passthroughLastFixTime = -1.0;

                    passthroughRetries = 0;

                }
            }

        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] Unexpected error in poll(): " + e.getMessage());
        }
    }

    private void fireADBRelaunch() {
        try {
            File adbFile = new File(ADB_PATH);
            if (!adbFile.exists()) {
                System.err.println("[QuestNavADBWatcher] adb not found at: " + ADB_PATH);
                return;
            }

            new ProcessBuilder(ADB_PATH, "-s", QUEST_ADB_ADDRESS,
                    "shell", "am", "start",
                    "-n", "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity")
                    .redirectErrorStream(true)
                    .start();

        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] Failed to execute ADB command: " + e.getMessage());
        }
    }

    private static double nowSeconds() {
        return System.nanoTime() / 1e9;
    }
}