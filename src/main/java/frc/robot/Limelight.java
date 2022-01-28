package frc.robot;
import java.lang.Math;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

class Limelight {
    private static NetworkTable limelightTable  = NetworkTableInstance.getDefault().getTable("limelight");

    /**
     * Calculates distance from target to limelight
     * @return distance from limelight
     */
    public static double estimateDistance() {
        int dif = Constants.LIMELIGHT_HEIGHT_OF_TARGET - Constants.LIMELIGHT_HEIGHT;
        double ty = limelightTable.getEntry("ty").getDouble(0);
        double distance = dif/Math.tan(Constants.LIMELIGHT_ANGLE + ty);
        return distance;
    }

    /**
     * retrieves current horizontal angle produced from limelight
     * @return limelight network table horizontal angle
     */
    public static double getHorizontalAngle(){
        return limelightTable.getEntry("tx").getDouble(0);
    }

    /**
     * switches limelight led to on or off depending on current state
     * @param mode true to set on or false to set off
     */
    public static void setLedOn(boolean mode){
        limelightTable.getEntry("ledMode").setNumber(mode ? 3 : 1);
    }  
}