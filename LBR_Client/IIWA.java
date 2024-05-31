package photocatalysis;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.task.ITaskLogger;


public class IIWA {
	private static Frame drive_position_tool = new Frame(10.871016176842172, -451.4392471782181, 148.18839400226915, Math.PI/2, 0.0, -Math.PI);
	
	public static void moveToDrivePos(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			_log.info("Movement to DrivePosition started");
			gripper.attachTo(iiwa.getFlange());
			gripper.getFrame("/TCP1").move(ptp(drive_position_tool).setJointVelocityRel(0.4));
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		} finally {
			_log.info("Movement to DrivePosition finished");
		}
	}
	
	public static void moveToHomePos(LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			_log.info("Movement to HomePos started");
			iiwa.move(ptpHome().setJointVelocityRel(0.1));
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		} finally {
			_log.info("Movement to HomePos finished");
		}
	}
	
	
}
