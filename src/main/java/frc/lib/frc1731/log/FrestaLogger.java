package frc.lib.frc1731.log;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotConstants;

/**
 * Class to handle AdvantageKit logging to USB and NetworkTables
 */
public class FrestaLogger {
    public static void start() {
        /*
		 * Note: do not think this is implemented in the gradle build, if we want to
		 * print this we will need to carry that over
		 */
		SmartDashboard.putString("Build Info - Branch", "N/A");
		SmartDashboard.putString("Build Info - Commit Hash", "N/A");
		SmartDashboard.putString("Build Info - Date", "N/A");

		/*
		 * Note: do not think this is implemented in the gradle build, if we want to
		 * print this we will need to carry that over
		 */
		try {
			File buildInfoFile = new File(Filesystem.getDeployDirectory(), "DeployedBranchInfo.txt");
			if (buildInfoFile.exists() && buildInfoFile.canRead()) {
				Scanner reader = new Scanner(buildInfoFile);
				int i = 0;
				while (reader.hasNext()) {
					if (i == 0) {
						SmartDashboard.putString("Build Info - Branch", reader.nextLine());
					} else if (i == 1) {
						SmartDashboard.putString("Build Info - Commit Hash", reader.nextLine());
					} else {
						SmartDashboard.putString("Build Info - Date", reader.nextLine());
					}
					i++;
				}
				reader.close();
			}
		} catch (FileNotFoundException fnf) {
			System.err.println("DeployedBranchInfo.txt not found");
			fnf.printStackTrace();
		}
		
		SmartDashboard.updateValues();

		Logger.start();
		if (Robot.isReal()) {
			Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
		} else if (RobotConstants.kLogToAdvantageScope){
			Logger.addDataReceiver(new NT4Publisher());
		}
    }
}