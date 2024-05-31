package photocatalysis;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import javax.inject.Inject;
import com.kuka.common.ThreadUtil;
import com.kuka.nav.task.NavTaskCategory;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.task.ITaskLogger;
import com.kuka.task.RoboticsAPITask;


public class MovementToDetectionPosition extends RoboticsAPIApplication {
	@Inject
	private LBR iiwa;
	
	@Inject
	private ITaskLogger _log;
	
	Frame camera_detection_tables = new Frame(-99.5457508888813, -361.89524071946016, 167.21127459458754, 0, -Math.PI/2, Math.PI);

	@Override
	public void initialize() {	
	}
	
	@Override
	public void run() throws Exception {
		iiwa.move(ptp(camera_detection_tables).setJointVelocityRel(0.1));
		ThreadUtil.milliSleep(3000);
	}
}
