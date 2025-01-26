package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.Config.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Simulator;
import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.enums.PortType;
import org.firstinspires.ftc.teamcode.sim.BlinkinSim;
import org.firstinspires.ftc.teamcode.sim.ColorSim;
import org.firstinspires.ftc.teamcode.sim.DistanceSim;
import org.firstinspires.ftc.teamcode.sim.IMUSim;
import org.firstinspires.ftc.teamcode.sim.MagnetSim;
import org.firstinspires.ftc.teamcode.sim.MotorSim;
import org.firstinspires.ftc.teamcode.sim.PotentSim;
import org.firstinspires.ftc.teamcode.sim.ServoSim;
import org.firstinspires.ftc.teamcode.sim.TouchSim;

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
            HardwareType.IMU, PortType.I2C
    ));

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

    public static double[] blankArray(double num) {
        return new double[(int) num];
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
            default:
                throw new UnknownHardwareType("Unknown HardwareType: " + type);
        }
    }
    public static Map<HardwareType, Class<? extends Simulator>> simulatorMap = new EnumMap<>(HardwareType.class);
    static{
        simulatorMap.put(HardwareType.MOTOR, MotorSim.class);
        simulatorMap.put(HardwareType.SERVO, ServoSim.class);
        simulatorMap.put(HardwareType.BLINKIN, BlinkinSim.class);
        simulatorMap.put(HardwareType.TOUCH_SENSOR, TouchSim.class);
        simulatorMap.put(HardwareType.POTENTIOMETER, PotentSim.class);
        simulatorMap.put(HardwareType.COLOR_SENSOR, ColorSim.class);
        simulatorMap.put(HardwareType.DISTANCE_SENSOR, DistanceSim.class);
        simulatorMap.put(HardwareType.MAGNETIC_LIMIT_SWITCH, MagnetSim.class);
        simulatorMap.put(HardwareType.IMU, IMUSim.class);
    }
    public static Class<? extends Simulator> getSimClass(HardwareType type) {
        return simulatorMap.get(type);
    }
    public static HardwareType getHardwareType(Simulator simulator){
        if(simulator instanceof MotorSim){
            return HardwareType.MOTOR;
        }else if(simulator instanceof ServoSim){
            return HardwareType.SERVO;
        }else if(simulator instanceof BlinkinSim){
            return HardwareType.BLINKIN;
        }else if(simulator instanceof TouchSim){
            return HardwareType.TOUCH_SENSOR;
        }else if(simulator instanceof PotentSim){
            return HardwareType.POTENTIOMETER;
        }else if(simulator instanceof ColorSim){
            return HardwareType.COLOR_SENSOR;
        }else if(simulator instanceof DistanceSim){
            return HardwareType.DISTANCE_SENSOR;
        }else if(simulator instanceof MagnetSim){
            return HardwareType.MAGNETIC_LIMIT_SWITCH;
        }else if(simulator instanceof IMUSim){
            return HardwareType.IMU;
        }else{
            throw new UnknownSim("Unknown Simulator: " + simulator);
        }
    }

    public static String approveIMU(IMU imu) {
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

    public static void approve(LinearOpMode opMode) {
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
                throw new TooManyDevices("Too many " + entry.getValue().name());
            }
        }
    }

    static double[] motorPowers = {};
    static double[] motorPositions = {};
    public static String recommendMotor(com.qualcomm.robotcore.hardware.DcMotor motor){
        String recommend = "";
        double pose = motor.getCurrentPosition();
        double power = motor.getPower();
        double powerDif = power - motorPowers[motorPowers.length - 1];
        double poseDif = pose - motorPositions[motorPositions.length - 1];
        // if motor power increases, so should position
        // if motor power decreases, so should position
        if (poseDif > 0 && powerDif >0){
            recommend+="Good";
        }else if (poseDif < 0 && powerDif < 0){
            recommend+="Good";
        }else if (poseDif > 0 && powerDif < 0){
            recommend+="Pose increased but power is decreasing";
        }else if (poseDif < 0 && powerDif > 0){
            recommend+="Pose decreased but power is increasing";
        }
        return recommend;
    }
}
