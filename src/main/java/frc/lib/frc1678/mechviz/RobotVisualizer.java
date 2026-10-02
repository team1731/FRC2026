package frc.lib.frc1678.mechviz;

import frc.lib.frc1678.util.Loopable;

import java.util.ArrayList;
import java.util.List;

public abstract class RobotVisualizer implements Loopable {
	/** Configure one robot using suppliers; none() remains a zero-work alternative. */
	public static RobotVisualization.Builder builder(String name) {
		return new RobotVisualization.Builder(name);
	}
	private final List<VisualizedRobot> registeredRobots = new ArrayList<>();

	public static RobotVisualizer none() {
		return new RobotVisualizer() {
			@Override
			public void registerRobots() {}
		};
	}

	public void registerRobot(VisualizedRobot robot) {
		registeredRobots.add(robot);
	}

	public abstract void registerRobots();

	public void init() {
		registerRobots();
	}

	@Override
	public void loop() {
		for (VisualizedRobot robot : registeredRobots) {
			robot.loop();
		}
	}
}
