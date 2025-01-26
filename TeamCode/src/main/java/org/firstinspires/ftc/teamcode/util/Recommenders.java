package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.IMU;

public class Recommenders {
    static double[] motorPowers = {};
    static double[] motorPositions = {};

    public static String recommendMotor(com.qualcomm.robotcore.hardware.DcMotor motor) {
        String recommend = "";
        double pose = motor.getCurrentPosition();
        double power = motor.getPower();
        double powerDif = power - motorPowers[motorPowers.length - 1];
        double poseDif = pose - motorPositions[motorPositions.length - 1];
        // if motor power increases, so should position
        // if motor power decreases, so should position
        if (poseDif > 0 && powerDif > 0) {
            recommend += "Good";
        } else if (poseDif < 0 && powerDif < 0) {
            recommend += "Good";
        } else if (poseDif > 0 && powerDif < 0) {
            recommend += "Pose increased but power is decreasing";
        } else if (poseDif < 0 && powerDif > 0) {
            recommend += "Pose decreased but power is increasing";
        }
        return recommend;
    }

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
