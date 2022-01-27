package frc.robot;
import java.lang.Math;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

class Limelight {
    private static NetworkTable tableNet  = NetworkTableInstance.getDefault().getTable("limelight");
    public static double estimateDistance() {
        int dif = Constants.LIMELIGHT_HEIGHT_OF_TARGET - Constants.LIMELIGHT_HEIGHT;
        double ty = tableNet.getEntry("ty").getDouble(0);
        double distance = dif/Math.tan(Constants.LIMELIGHT_ANGLE + ty);
        return distance;
    }
    public static double getHorizontalAngle(){
        return tableNet.getEntry("tx").getDouble(0);
    }
    
}