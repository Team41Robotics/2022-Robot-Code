package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Custom class to manage the networktable interface of PhotonVision
 */
public class PhotonCamera {
    public static NetworkTable driverCam = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("FrontCamera");

    /**
     * Get the area the current target takes up
     * @return the area (idk what units)
     */
    public static double getArea() {
        return driverCam.getEntry("targetArea").getDouble(0);
    }

    /**
     * Get the yaw offset of the current target in relation to the camera
     * @return the yaw offset in degrees
     */
    public static double getYaw() {
        return driverCam.getEntry("targetYaw").getDouble(0);
    }

    /**
     * Get if the camera has a target or not
     * @return True if a target is found, false if not
     */
    public static boolean hasTarget() {
        return driverCam.getEntry("hasTarget").getBoolean(false);
    }

    /**
     * Set the photonvision pipeline to the correct one
     * @param team true for blue, false for red
     */
    public static void setPipeline(boolean team) {
        driverCam.getEntry("pipelineIndex").setDouble(team ? 3 : 2);
    }

    /**
     * Upload the telemetry for PhotonVision
     * @param table the base telemetry NetworkTable
     */
    public static void telemetry(NetworkTable table) {
        table.getEntry("angle").setDouble(getYaw());
        table.getEntry("area").setDouble(getArea());
    }
}
