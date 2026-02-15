package frc.robot;

import java.io.File;
import java.util.*;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;

public class AutoLoader {
    private static SendableChooser<String> autoChooser = new SendableChooser<>();
    private static HashMap<String, String> autoPaths;

    public AutoLoader() {
        String[] autoModes = getAutoModes();
		for (String autoMode : autoModes) {
			autoChooser.addOption(autoMode, autoMode);
		}

		// pre-load the default auto
        String defaultAuto = Constants.AutoConstants.kAutoDefault;
		autoChooser.setDefaultOption(defaultAuto, defaultAuto);

        // Put on SmartDashboard
        SmartDashboard.putData(AutoConstants.kAutoCodeKey, autoChooser);
    }

    private static String[] getAutoModes() {
        autoPaths = findPaths(new File(Filesystem.getLaunchDirectory(),
            (Robot.isReal() ? "/home/lvuser" : "/src/main") + "/deploy/pathplanner/autos"));

        List<String> autoModes = new ArrayList<String>();
        for (String key : autoPaths.keySet()) {
            String stripKey = key.toString();
            if (!autoModes.contains(stripKey)) {
                autoModes.add(stripKey);
            }
        }
        autoModes.sort((p1, p2) -> p1.compareTo(p2));
        return autoModes.toArray(String[]::new);
    }

    private static HashMap<String, String> findPaths(File directory) {
        HashMap<String, String> autoPaths = new HashMap<String, String>();
        if (!directory.exists()) {
            System.out.println("FATAL: path directory not found! " + directory.getAbsolutePath());
        } else {
            File[] files = directory.listFiles();
            if (files == null) {
                System.out.println("FATAL: I/O error or NOT a directory: " + directory);
            } else {
                for (File file : files) {
                    String fileName = file.getName();
                    String key = fileName.replace(".auto", "");
                    String path = file.getAbsolutePath();
                    System.out.println(path);
                    autoPaths.put(key, path);
                }
            }
        }
        return autoPaths;
    }

    public Command getSelectedAutoMode() {
        return new PathPlannerAuto(autoChooser.getSelected());
    }
}