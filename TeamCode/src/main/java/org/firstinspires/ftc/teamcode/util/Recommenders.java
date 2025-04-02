package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.IMU;

public class Recommenders {
    // This method recommends whether to keep or replace an IMU
    public static String recommendIMU(IMU imu) {
        String str = imu.getClass().getCanonicalName();
        String name = "";
        if (str.contains("BHI")) {
            name = "BHI055";
        } else if (str.contains("BNO")) {
            name = "BNO055";
        } else {
            name = "Unknown";
        }
        if (name == "BHI055") {
            return "Replace";
        } else if (name == "BNO055") {
            return "Keep";
        } else {
            return "Unknown IMU";
        }
    }
}
