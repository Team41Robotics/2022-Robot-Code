package frc.robot;
import java.lang.Math;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

class Limelight {
    private static NetworkTable limelightTable  = NetworkTableInstance.getDefault().getTable("limelight");

    /**
     * Calculates distance from target to limelight
     * @return distance from limelight
     */
    public static double estimateDistance() {
        double dif = Constants.LIMELIGHT_HEIGHT_OF_TARGET - Constants.LIMELIGHT_HEIGHT;
        double ty = limelightTable.getEntry("ty").getDouble(0);
        double distance = dif/Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE + ty));
        return distance-27;
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

    public static void test() {
        System.out.println(estimateDistance());
    }   
    
    public static double getRobotAngle(){
        double cameraAngle = Math.toRadians(getHorizontalAngle());
        double distance = estimateDistance();
        double x = Constants.LIMELIGHT_DEPTH_OFFSET + (distance * Math.cos(cameraAngle));
        double y = Constants.LIMELIGHT_HORIZONTAL_OFFSET + (distance * Math.sin(cameraAngle));
        return Math.toDegrees(Math.atan2(y, x));
    }

    public static boolean targetFound() {
        return getHorizontalAngle() != 0.0;
    }
    
    public static void manualZoom(Joystick secondDS) {
        int POVVal = secondDS.getPOV();
        if (POVVal == 0) {
            limelightTable.getEntry("pipeline").setNumber(0);
        } else {
            limelightTable.getEntry("pipeline").setNumber(1);
        }
    }

    public static void resetZoom() {
        limelightTable.getEntry("pipeline").setNumber(0);
    }

    public static void zoomIn() {
        limelightTable.getEntry("pipeline").setNumber(1);
    }
}