package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.Config.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.HardwareType;

public class Util {
    public static double count(HardwareType hardwareType) {
        return hardwareMap.entrySet().stream().filter(entry -> entry.getKey() == HardwareType.MOTOR).count();
    }

    public static double hubCount(HardwareMap hardwareMap) {
        return hardwareMap.getAll(com.qualcomm.hardware.lynx.LynxModule.class).size();
    }

    public static void progressTelemetry(Telemetry telemetry, double len, double progress) {
        telemetry.addLine(progressIndicator(len, progress));
        telemetry.addLine(progressPercent(len, progress));
    }

    public static String progressIndicator(double len, double progress) {
        return (int) progress + "/" + (int) len;
    }

    public static String progressPercent(double len, double progress) {
        return (int) (progress / len * 100) + "%";
    }

    public static double[] blankArray(double num) {
        return new double[(int) num];
    }
}
