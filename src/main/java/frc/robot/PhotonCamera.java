package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotonCamera {
    public static NetworkTable driverCam = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("HD_USB_Camera");

    public static double getArea() {
        return driverCam.getEntry("targetArea").getDouble(0);
    }

    public static double getYaw() {
        return driverCam.getEntry("targetYaw").getDouble(0);
    }

    public static boolean hasTarget() {
        return driverCam.getEntry("hasTarget").getBoolean(false);
    }
}
