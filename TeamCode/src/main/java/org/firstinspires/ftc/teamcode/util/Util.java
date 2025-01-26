package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.Config.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Simulator;
import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.enums.PortType;
import org.firstinspires.ftc.teamcode.sim.BlinkinSim;
import org.firstinspires.ftc.teamcode.sim.CameraSim;
import org.firstinspires.ftc.teamcode.sim.ColorSim;
import org.firstinspires.ftc.teamcode.sim.DistanceSim;
import org.firstinspires.ftc.teamcode.sim.IMUSim;
import org.firstinspires.ftc.teamcode.sim.MagnetSim;
import org.firstinspires.ftc.teamcode.sim.MotorSim;
import org.firstinspires.ftc.teamcode.sim.PotentSim;
import org.firstinspires.ftc.teamcode.sim.ServoSim;
import org.firstinspires.ftc.teamcode.sim.TouchSim;
import org.firstinspires.ftc.teamcode.util.exceptions.UnknownHardwareType;
import org.firstinspires.ftc.teamcode.util.exceptions.UnknownSim;

import java.util.EnumMap;
import java.util.Map;

public class Util {
    public static Map<HardwareType, PortType> portMap = new EnumMap<>(Map.of(
            HardwareType.MOTOR, PortType.MOTOR,
            HardwareType.SERVO, PortType.SERVO,
            HardwareType.BLINKIN, PortType.SERVO,
            HardwareType.TOUCH_SENSOR, PortType.DIGITAL,
            HardwareType.POTENTIOMETER, PortType.ANALOG,
            HardwareType.COLOR_SENSOR, PortType.I2C,
            HardwareType.DISTANCE_SENSOR, PortType.I2C,
            HardwareType.MAGNETIC_LIMIT_SWITCH, PortType.DIGITAL,
            HardwareType.IMU, PortType.I2C,
            HardwareType.CAMERA, PortType.USB
    ));
    public static Map<HardwareType, Class<? extends Simulator>> simulatorMap = new EnumMap<>(HardwareType.class);

    static {
        simulatorMap.put(HardwareType.MOTOR, MotorSim.class);
        simulatorMap.put(HardwareType.SERVO, ServoSim.class);
        simulatorMap.put(HardwareType.BLINKIN, BlinkinSim.class);
        simulatorMap.put(HardwareType.TOUCH_SENSOR, TouchSim.class);
        simulatorMap.put(HardwareType.POTENTIOMETER, PotentSim.class);
        simulatorMap.put(HardwareType.COLOR_SENSOR, ColorSim.class);
        simulatorMap.put(HardwareType.DISTANCE_SENSOR, DistanceSim.class);
        simulatorMap.put(HardwareType.MAGNETIC_LIMIT_SWITCH, MagnetSim.class);
        simulatorMap.put(HardwareType.IMU, IMUSim.class);
        simulatorMap.put(HardwareType.CAMERA, CameraSim.class);
    }

    public static double count(HardwareType hardwareType) {
        return hardwareMap.entrySet().stream().filter(entry -> entry.getValue() == hardwareType).count();
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

    public static java.lang.Class getClass(HardwareType type) {
        switch (type) {
            case MOTOR:
                return com.qualcomm.robotcore.hardware.DcMotor.class;
            case SERVO:
                return com.qualcomm.robotcore.hardware.Servo.class;
            case IMU:
                return com.qualcomm.robotcore.hardware.IMU.class;
            case BLINKIN:
                return com.qualcomm.hardware.rev.RevBlinkinLedDriver.class;
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
            case CAMERA:
                return WebcamName.class;
        }
        return null;
    }

    public static Simulator getSimulator(HardwareType type, LinearOpMode opMode) {
        switch (type) {
            case MOTOR:
                return new MotorSim(opMode);
            case SERVO:
                return new ServoSim(opMode);
            case BLINKIN:
                return new BlinkinSim(opMode);
            case TOUCH_SENSOR:
                return new TouchSim(opMode);
            case POTENTIOMETER:
                return new PotentSim(opMode);
            case COLOR_SENSOR:
                return new ColorSim(opMode);
            case DISTANCE_SENSOR:
                return new DistanceSim(opMode);
            case MAGNETIC_LIMIT_SWITCH:
                return new MagnetSim(opMode);
            case IMU:
                return new IMUSim(opMode);
            case CAMERA:
                return new CameraSim(opMode);
            default:
                throw new UnknownHardwareType("Unknown HardwareType: " + type);
        }
    }

    public static Class<? extends Simulator> getSimClass(HardwareType type) {
        return simulatorMap.get(type);
    }

    public static HardwareType getHardwareType(Simulator simulator) {
        if (simulator instanceof MotorSim) {
            return HardwareType.MOTOR;
        } else if (simulator instanceof ServoSim) {
            return HardwareType.SERVO;
        } else if (simulator instanceof BlinkinSim) {
            return HardwareType.BLINKIN;
        } else if (simulator instanceof TouchSim) {
            return HardwareType.TOUCH_SENSOR;
        } else if (simulator instanceof PotentSim) {
            return HardwareType.POTENTIOMETER;
        } else if (simulator instanceof ColorSim) {
            return HardwareType.COLOR_SENSOR;
        } else if (simulator instanceof DistanceSim) {
            return HardwareType.DISTANCE_SENSOR;
        } else if (simulator instanceof MagnetSim) {
            return HardwareType.MAGNETIC_LIMIT_SWITCH;
        } else if (simulator instanceof IMUSim) {
            return HardwareType.IMU;
        } else if (simulator instanceof CameraSim) {
            return HardwareType.CAMERA;
        } else {
            throw new UnknownSim("Unknown Simulator: " + simulator);
        }
    }


}
