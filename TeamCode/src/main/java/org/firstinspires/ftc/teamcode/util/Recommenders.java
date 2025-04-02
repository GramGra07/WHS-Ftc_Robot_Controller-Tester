package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.IMU;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Recommenders {
    static List<Double> motorPowers = new ArrayList<>();
    static List<Double> motorPositions =new ArrayList<>();

    // This method recommends whether to keep or replace a motor as well as verifications and reasons
    private static String recommendMotor(com.qualcomm.robotcore.hardware.DcMotor motor) {
        String recommend = "";
        double last = 0;
        double lastPose = 0;
if (motorPowers.size()>0) {
    last = motorPowers.get(motorPowers.size() - 1);
    lastPose = motorPositions.get(motorPositions.size() - 1);
}
        double pose = motor.getCurrentPosition();
        double power = motor.getPower();
        motorPowers.add(power);
        motorPositions.add(pose);
        if (motorPowers.size()>0) {
            double powerDif = power - last;
            double poseDif = pose - lastPose;
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
        }else{
            recommend += "No previous data";
        }
        return recommend;
    }

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
