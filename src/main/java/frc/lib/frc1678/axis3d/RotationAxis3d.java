package frc.lib.frc1678.axis3d;

import javax.xml.transform.Source;

public enum RotationAxis3d implements Axis3dConvertable<Source> {
	ROLL,
	PITCH,
	YAW;

	@Override
	public Axis3d toAxis3d() {
		return switch (this) {
			case ROLL -> Axis3d.ROLL;
			case PITCH -> Axis3d.PITCH;
			case YAW -> Axis3d.YAW;
		};
	}
}
