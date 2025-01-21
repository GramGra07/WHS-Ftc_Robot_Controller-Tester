package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.Config.hardwareMap;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.enums.PortType;

import java.util.EnumMap;
import java.util.Map;

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

    public static java.lang.Class getClass(HardwareType type) {
        switch (type) {
            case MOTOR:
                return DcMotor.class;
            case SERVO:
                return Servo.class;
            case BLINKIN:
                return RevBlinkinLedDriver.class;
                case TOUCH_SENSOR:
                return com.qualcomm.robotcore.hardware.TouchSensor.class;
            case POTENTIOMETER:
                return com.qualcomm.robotcore.hardware.AnalogInput.class;
            case COLOR_SENSOR:
                return com.qualcomm.robotcore.hardware.ColorSensor.class;
            case DISTANCE_SENSOR:
                return com.qualcomm.robotcore.hardware.DistanceSensor.class;
            case MAGNETIC_LIMIT_SWITCH:
                return com.qualcomm.robotcore.hardware.DigitalChannel.class;
        }
        return null;
    }

    public static Map<HardwareType, PortType> portMap = new EnumMap<>(Map.of(
            HardwareType.MOTOR, PortType.MOTOR,
            HardwareType.SERVO, PortType.SERVO,
            HardwareType.BLINKIN, PortType.SERVO,
            HardwareType.TOUCH_SENSOR, PortType.DIGITAL,
            HardwareType.POTENTIOMETER, PortType.ANALOG,
            HardwareType.COLOR_SENSOR, PortType.I2C,
            HardwareType.DISTANCE_SENSOR, PortType.I2C,
            HardwareType.MAGNETIC_LIMIT_SWITCH, PortType.DIGITAL
    ));

    public static IllegalArgumentException approve(LinearOpMode opMode) {
        double motorLimit = 4;
        double servoLimit = 6;
        double i2cLimit = 4;
        double analogLimit = 4;
        double digitalLimit = 8;
        for (Map.Entry<HardwareType, PortType> entry : portMap.entrySet()) {
            double count = Util.count(entry.getKey());
            double limit = 0;
            double hubCount = Util.hubCount(opMode.hardwareMap);
            switch (entry.getValue()) {
                case MOTOR:
                    limit = motorLimit;
                    break;
                case SERVO:
                    limit = servoLimit;
                    break;
                case I2C:
                    limit = i2cLimit;
                    break;
                case ANALOG:
                    limit = analogLimit;
                    break;
                case DIGITAL:
                    limit = digitalLimit;
                    break;
            }
            limit *= hubCount;
            if (count > limit) {
                return new IllegalArgumentException("Too many " + entry.getValue().name());
            }
        }
        return null;
    }
}
