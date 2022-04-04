package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotonCamera {
    public static NetworkTable driverCam = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("FrontCamera");

    public static double getArea() {
        return driverCam.getEntry("targetArea").getDouble(0);
    }

    public static double getYaw() {
        return driverCam.getEntry("targetYaw").getDouble(0);
    }

    public static boolean hasTarget() {
        return driverCam.getEntry("hasTarget").getBoolean(false);
    }

    /**
     * True: Blue
     * False: Red
     */
    public static void setPipeline(boolean team) {
        if (team) {
            driverCam.getEntry("pipelineIndex").setDouble(3);
        } else {
            driverCam.getEntry("pipelineIndex").setDouble(2);
        }
    }
}
