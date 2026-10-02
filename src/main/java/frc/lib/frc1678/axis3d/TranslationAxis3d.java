package frc.lib.frc1678.axis3d;

import javax.xml.transform.Source;

public enum TranslationAxis3d implements Axis3dConvertable<Source> {
	X,
	Y,
	Z;

	public Axis3d toAxis3d() {
		return switch (this) {
			case X -> Axis3d.X;
			case Y -> Axis3d.Y;
			case Z -> Axis3d.Z;
		};
	}
}
