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
     * Set the photonvision pipeline to the correct one
     * @param team true for blue, false for red
     */
    public static void setPipeline(boolean team) {
        driverCam.getEntry("pipelineIndex").setDouble(team ? 3 : 2);
    }

    public static void telemetry(NetworkTable table) {
        table.getEntry("angle").setDouble(getYaw());
        table.getEntry("area").setDouble(getArea());
    }
}
