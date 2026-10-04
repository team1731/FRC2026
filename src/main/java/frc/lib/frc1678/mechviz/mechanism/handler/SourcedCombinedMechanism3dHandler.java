package frc.lib.frc1678.mechviz.mechanism.handler;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.frc1678.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.frc1678.axis3d.Axis3d;
import frc.lib.frc1678.builder.Transform3dObjectBuilder;

public class SourcedCombinedMechanism3dHandler extends Mechanism3dHandler {
	private final Mechanism3d source;
	private final Pair<Mechanism3d, Axis3d[]> effected[];

	@SuppressWarnings({"unchecked", "varargs"})
	public SourcedCombinedMechanism3dHandler(Mechanism3d source, Pair<Mechanism3d, Axis3d[]>... effected) {
		super();

		this.source = source;
		this.effected = effected;
		registerMech(source);
		for (Pair<Mechanism3d, ?> mech : effected) {
			registerMech(mech.getFirst());
		}
	}

	@Override
	public void updatePositions() {
		Transform3d sourceTransform = source.getMutationOffset();

		for (Pair<Mechanism3d, Axis3d[]> pair : effected) {
			Mechanism3d mech = pair.getFirst();
			Axis3d travelAxes[] = pair.getSecond();

			if (travelAxes.length == 0) {
				continue;
			}

			Transform3dObjectBuilder combined = new Transform3dObjectBuilder(mech.getMutationOffset());
			for (Axis3d axis : travelAxes) {
				switch (axis) {
					case X -> combined.withX(sourceTransform.getMeasureX());
					case Y -> combined.withY(sourceTransform.getMeasureY());
					case Z -> combined.withZ(sourceTransform.getMeasureZ());
					case ROLL -> combined.withRoll(sourceTransform.getRotation().getMeasureX());
					case PITCH -> combined.withPitch(sourceTransform.getRotation().getMeasureY());
					case YAW -> combined.withYaw(sourceTransform.getRotation().getMeasureZ());
				}
			}
			mech.setMutatedTransform(combined.build());
		}
	}
}
