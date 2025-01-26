package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Config.hardwareMap;
import static org.firstinspires.ftc.teamcode.util.Util.approve;
import static org.firstinspires.ftc.teamcode.util.Util.getHardwareType;
import static org.firstinspires.ftc.teamcode.util.Util.getSimClass;
import static org.firstinspires.ftc.teamcode.util.Util.recommendMotor;

import android.util.Pair;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.util.HardwareDeviceNotFound;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

public class Robot {
    private static final Map<HardwareType, ArrayList<String>> deviceMap = new EnumMap<>(HardwareType.class);

    private final Map<String, Pair<HardwareDevice, HardwareType>> hardMap = new HashMap<>();
    private final LinearOpMode opMode;
    private RevBlinkinLedDriver.BlinkinPattern blinkinPattern;

    public Robot(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }
    private void setupMaps(){
        deviceMap.clear();
        hardMap.clear();
        for (HardwareType hardwareType : HardwareType.values()) {
            deviceMap.put(hardwareType, new ArrayList<>());
        }

        for (Map.Entry<String, HardwareType> entry : hardwareMap.entrySet()) {
            HardwareType hardwareType = entry.getValue();
            String hardwareName = entry.getKey();
            deviceMap.get(hardwareType).add(hardwareName);
        }
    }

    public void init(Simulator simulator) {
        approve(opMode);
        setupMaps();
        checkCount(getHardwareType(simulator));
        for (Map.Entry<String, HardwareType> entry : hardwareMap.entrySet()) {
            HardwareType hardwareType = entry.getValue();
            String hardwareName = entry.getKey();
            hardMap.put(hardwareName, new Pair(opMode.hardwareMap.get(Util.getClass(hardwareType), hardwareName), hardwareType));
            initDevice(simulator, hardwareName);
        }
    }

    public void checkCount(HardwareType type){
        double count = count(type);
        if (count == 0) {
            throw new HardwareDeviceNotFound("No " + type + " found in hardwareMap");
        }
    }

    public double count(HardwareType hardwareType) {
        return deviceMap.get(hardwareType).size();
    }

    public HardwareType getType(String name) {
        return hardMap.get(name).second;
    }

    public <T extends HardwareDevice> T getHardware(String name, Class<T> cast) {
        return cast.cast(hardMap.get(name).first);
    }

    public void initDevice(Simulator simulator, String hardwareName) {
        if (!getSimClass(getType(hardwareName)).equals(simulator.getClass())) return;
        switch (getType(hardwareName)) {
            case MOTOR:
                (getHardware(hardwareName, DcMotor.class)).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case SERVO:
                (getHardware(hardwareName, Servo.class)).setPosition(0);
                break;
            case BLINKIN:
                (getHardware(hardwareName, RevBlinkinLedDriver.class)).setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case IMU:
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                (getHardware(hardwareName, IMU.class)).initialize(parameters);
                break;
        }
    }


    public void setPower(double power) {
        for (int i = 0; i < count(HardwareType.MOTOR); i++) {
            String name = deviceMap.get(HardwareType.MOTOR).get(i);
            DcMotor motor = getHardware(name, DcMotor.class);
            motor.setPower(power);
        }
    }

    public void setPosition(double position) {
        for (int i = 0; i < count(HardwareType.SERVO); i++) {
            String name = deviceMap.get(HardwareType.SERVO).get(i);
            Servo servo = getHardware(name, Servo.class);
            servo.setPosition(position);
        }
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinPattern = pattern;
        for (int i = 0; i < count(HardwareType.BLINKIN); i++) {
            String name = deviceMap.get(HardwareType.BLINKIN).get(i);
            RevBlinkinLedDriver blinkin = getHardware(name, RevBlinkinLedDriver.class);
            blinkin.setPattern(pattern);
        }
    }

    public void imuTelemetry() {
        for (int i = 0; i < count(HardwareType.IMU); i++) {
            String name = deviceMap.get(HardwareType.IMU).get(i);
            IMU imu = getHardware(name, IMU.class);
            opMode.telemetry.addData(name + " Angles", imu.getRobotYawPitchRollAngles());
            opMode.telemetry.addData(name + " Type", imu.getClass().getCanonicalName());
            opMode.telemetry.addData(name + " Recommendation ", Util.approveIMU(imu));
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }

    public void motorTelemetry() {
        for (int i = 0; i < count(HardwareType.MOTOR); i++) {
            String name = deviceMap.get(HardwareType.MOTOR).get(i);
            DcMotor motor = getHardware(name, DcMotor.class);
            opMode.telemetry.addData(name + " Power", motor.getPower());
            opMode.telemetry.addData(name + " Position", motor.getCurrentPosition());
            opMode.telemetry.addLine(recommendMotor(motor));
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }

    public void blinkTelemetry() {
        for (int i = 0; i < count(HardwareType.BLINKIN); i++) {
            String name = deviceMap.get(HardwareType.BLINKIN).get(i);
            opMode.telemetry.addData(name + " Pattern", blinkinPattern);
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }

    public void servoTelemetry() {
        for (int i = 0; i < count(HardwareType.SERVO); i++) {
            String name = deviceMap.get(HardwareType.SERVO).get(i);
            Servo servo = getHardware(name, Servo.class);
            opMode.telemetry.addData(name + " Position", servo.getPosition());
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }

    public void colorTelemetry() {
        for (int i = 0; i < count(HardwareType.COLOR_SENSOR); i++) {
            String name = deviceMap.get(HardwareType.COLOR_SENSOR).get(i);
            com.qualcomm.robotcore.hardware.ColorSensor colorSensor = getHardware(name, com.qualcomm.robotcore.hardware.ColorSensor.class);
            opMode.telemetry.addData(name + " Red", colorSensor.red());
            opMode.telemetry.addData(name + " Green", colorSensor.green());
            opMode.telemetry.addData(name + " Blue", colorSensor.blue());
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }

    public void distanceTelemetry() {
        for (int i = 0; i < count(HardwareType.DISTANCE_SENSOR); i++) {
            String name = deviceMap.get(HardwareType.DISTANCE_SENSOR).get(i);
            com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor = getHardware(name, com.qualcomm.robotcore.hardware.DistanceSensor.class);
            opMode.telemetry.addData(name + " (cm) Distance", distanceSensor.getDistance(DistanceUnit.CM));
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }

    public void potentTelemetry() {
        for (int i = 0; i < count(HardwareType.POTENTIOMETER); i++) {
            String name = deviceMap.get(HardwareType.POTENTIOMETER).get(i);
            com.qualcomm.robotcore.hardware.AnalogInput potentiometer = getHardware(name, com.qualcomm.robotcore.hardware.AnalogInput.class);
            opMode.telemetry.addData(name + " Voltage", potentiometer.getVoltage());
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }

    public void touchTelemetry() {
        for (int i = 0; i < count(HardwareType.TOUCH_SENSOR); i++) {
            String name = deviceMap.get(HardwareType.TOUCH_SENSOR).get(i);
            com.qualcomm.robotcore.hardware.TouchSensor touchSensor = getHardware(name, com.qualcomm.robotcore.hardware.TouchSensor.class);
            opMode.telemetry.addData(name + " Pressed", touchSensor.isPressed());
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }

    public void magnetTelemetry() {
        for (int i = 0; i < count(HardwareType.MAGNETIC_LIMIT_SWITCH); i++) {
            String name = deviceMap.get(HardwareType.MAGNETIC_LIMIT_SWITCH).get(i);
            com.qualcomm.robotcore.hardware.DigitalChannel magneticSwitch = getHardware(name, com.qualcomm.robotcore.hardware.DigitalChannel.class);
            opMode.telemetry.addData(name + " Pressed", magneticSwitch.getState());
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }

}
