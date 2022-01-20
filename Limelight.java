import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
public static double estimateDistance() {
    int dif = Constants.LIMELIGHT_HEIGHT_OF_TARGET - Constants.LIMELIGHT_HEIGHT;
    double ty =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double distance = dif/Math.tan(Constants.LIMELIGHT_ANGLE + ty);
    return distance;

}