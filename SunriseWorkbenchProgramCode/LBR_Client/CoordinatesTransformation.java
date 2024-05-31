package photocatalysis;

import com.kuka.roboticsAPI.geometricModel.Frame;

public class CoordinatesTransformation {
	public static Frame transformCoordsToFlange(Frame parentFrame, double x_point, double y_point, double z_point, double a_point, double b_point, double c_point) {
		double x_point_new = -y_point;
		double y_point_new = -x_point;
		double z_point_new = -z_point;
		double a_point_new = -a_point;
		double b_point_new = -c_point;
		double c_point_new = -b_point;
		Frame frame = new Frame(parentFrame, x_point_new, y_point_new, z_point_new, a_point_new, b_point_new, c_point_new);
		return frame;
	}
}
