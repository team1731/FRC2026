package frc.lib.frc1678.mechviz;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.ErrorMessages;
import frc.lib.frc1678.mechviz.mechanism.DevelopmentHandler;
import frc.lib.frc1678.mechviz.mechanism.MechanismRegistry;
import frc.lib.frc1678.mechviz.mechanism.handler.Mechanism3dHandler;
import frc.lib.frc1678.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.frc1678.mechviz.mechanism.mech.PivotingMechanism3d;
import frc.lib.frc1678.util.Loopable;

import java.util.function.BiFunction;

public class VisualizedRobot implements Loopable {
	private final String name;

	private final MechanismRegistry registry;

	protected final DevelopmentHandler devHandler;

	public VisualizedRobot(String name, NetworkTable publishingTable, String debugTableSubEntry) {
		this.name = name;
		this.registry = new MechanismRegistry(publishingTable.getSubTable(name));
		devHandler = new DevelopmentHandler(publishingTable.getSubTable(debugTableSubEntry));
	}

	public String getName() {
		return this.name;
	}

	public void registerMechanism(String key, Mechanism3d mech) {
		registry.registerPublishedMechanism(key, mech);
		registry.addLoop(mech);
	}

	public void registerMechanism(BiFunction<Integer, Mechanism3d, String> nameFunction, Mechanism3dHandler handler) {
		registry.addLoop(handler);
		handler.foreachRegistered((index, mech) -> registerMechanism(nameFunction.apply(index, mech), mech));
	}

	public void setDrive(String key, PivotingMechanism3d drive) {
		ErrorMessages.requireNonNullParam(key, "key", "setDrive");
		ErrorMessages.requireNonNullParam(drive, "drive", "setDrive");
		registerMechanism(key, drive);
	}

	public void setDrive(PivotingMechanism3d drive) {
		ErrorMessages.requireNonNullParam(drive, "drive", "setDrive");
		setDrive("Drive", drive);
	}

	@Override
	public void loop() {
		registry.loop();
	}
}