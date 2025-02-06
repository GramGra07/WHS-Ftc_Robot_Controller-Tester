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
import org.firstinspires.ftc.teamcode.sim.EncoderSim;
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
    // maps port type to the hardware type
    public static Map<HardwareType, PortType> portMap = new EnumMap<>(HardwareType.class);
    static{
        portMap.put(HardwareType.MOTOR, PortType.MOTOR);
        portMap.put(HardwareType.SERVO, PortType.SERVO);
        portMap.put(HardwareType.BLINKIN, PortType.SERVO);
        portMap.put(HardwareType.TOUCH_SENSOR, PortType.DIGITAL);
        portMap.put(HardwareType.POTENTIOMETER, PortType.ANALOG);
        portMap.put(HardwareType.COLOR_SENSOR, PortType.I2C);
        portMap.put(HardwareType.DISTANCE_SENSOR, PortType.I2C);
        portMap.put(HardwareType.MAGNETIC_LIMIT_SWITCH, PortType.DIGITAL);
        portMap.put(HardwareType.IMU, PortType.I2C);
        portMap.put(HardwareType.CAMERA, PortType.USB);
        portMap.put(HardwareType.EXTERNAL_ENCODER, PortType.MOTOR);
    }
    // maps the testing simulator to the hardware type
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
        simulatorMap.put(HardwareType.EXTERNAL_ENCODER, EncoderSim.class);
    }

    // counts the number of devices of a certain type
    public static double count(HardwareType hardwareType) {
        return hardwareMap.entrySet().stream().filter(entry -> entry.getValue() == hardwareType).count();
    }

    // counts the number of hubs, 2 if e and c hubs are used
    public static double hubCount(HardwareMap hardwareMap) {
        return hardwareMap.getAll(com.qualcomm.hardware.lynx.LynxModule.class).size();
    }

    // updates the telemetry with the progress of the test
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

    // gets the class of the hardware type
    public static java.lang.Class getClass(HardwareType type) {
        switch (type) {
            case MOTOR:
            case EXTERNAL_ENCODER:
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

    // gets the simulator of the hardware type
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
            case EXTERNAL_ENCODER:
                return new EncoderSim(opMode);
            default:
                throw new UnknownHardwareType("Unknown HardwareType: " + type);
        }
    }

    // gets the simulator class of the hardware type
    public static Class<? extends Simulator> getSimClass(HardwareType type) {
        return simulatorMap.get(type);
    }

    // gets the hardware type of the simulator
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
        } else if (simulator instanceof EncoderSim){
            return HardwareType.EXTERNAL_ENCODER;
        }
        else {
            throw new UnknownSim("Unknown Simulator: " + simulator);
        }
    }


}
