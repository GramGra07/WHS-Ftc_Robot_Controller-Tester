package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Config.hardwareMap;
import static org.firstinspires.ftc.teamcode.util.Approvals.approve;
import static org.firstinspires.ftc.teamcode.util.Recommenders.recommendIMU;
import static org.firstinspires.ftc.teamcode.util.Recommenders.recommendMotor;
import static org.firstinspires.ftc.teamcode.util.Util.getHardwareType;
import static org.firstinspires.ftc.teamcode.util.Util.getSimClass;

import android.util.Pair;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.util.exceptions.HardwareDeviceNotFound;
import org.firstinspires.ftc.teamcode.util.exceptions.HardwareNotFoundOnRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.gentrifiedApps.gentrifiedAppsUtil.velocityVision.pipelines.bow.BlackAndWhiteDotDetector;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

public class Robot {
    // This is a map of hardware types with the names of each device
    public static final Map<HardwareType, ArrayList<String>> deviceMap = new EnumMap<>(HardwareType.class);
    // This is a map of hardware names (unique) with the device initialized and the type of hardware
    private final Map<String, Pair<HardwareDevice, HardwareType>> hardMap = new HashMap<>();
    private final LinearOpMode opMode; // generic opmode declaration
    private RevBlinkinLedDriver.BlinkinPattern blinkinPattern; // pattern for blinkin

    public Robot(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    private void setupMaps() { // sets up the device map, creates device and hard maps to be used further
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

    // initialized the robot with the current simulator
    public void init(Simulator simulator) {
        approve(opMode);
        setupMaps();
        checkCount(getHardwareType(simulator));
        for (Map.Entry<String, HardwareType> entry : hardwareMap.entrySet()) {
            HardwareType hardwareType = entry.getValue();
            String hardwareName = entry.getKey();
            try {
                hardMap.put(hardwareName, new Pair(opMode.hardwareMap.get(Util.getClass(hardwareType), hardwareName), hardwareType));
            }catch (NullPointerException e) {
                throw new HardwareNotFoundOnRobot("No " + hardwareName + " found in hardwareMap configuration");
            }
            initDevice(simulator, hardwareName);
        }
    }

    // simple check to see if the hardware type is present when initializing the robot
    public void checkCount(HardwareType type) {
        double count = count(type);
        if (count == 0) {
            throw new HardwareDeviceNotFound("No " + type + " found in hardwareMap");
        }
    }

    // Returns the count of the hardware type
    public double count(HardwareType hardwareType) {
        return deviceMap.get(hardwareType).size();
    }

    // Returns the hardware type of the device with the name
    public HardwareType getType(String name) {
        return hardMap.get(name).second;
    }

    // Returns the hardware device with the name and type, casted to the correct type
    public <T extends HardwareDevice> T getHardware(String name, Class<T> cast) {
        return cast.cast(hardMap.get(name).first);
    }

    // Initializes the devices only for the current simulator
    public void initDevice(Simulator simulator, String hardwareName) {
        if (!getSimClass(getType(hardwareName)).equals(simulator.getClass())) return;
        switch (getType(hardwareName)) {
            case MOTOR:
            case EXTERNAL_ENCODER:
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
            case CAMERA:
                VisionPortal portal = new VisionPortal.Builder()
                        .setCamera(getHardware(hardwareName, WebcamName.class))
                        .addProcessor(new AprilTagProcessor.Builder().build())
                        .build();
                break;
        }
    }

    //Setters for individual hardware types


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

    // Telemetry methods for each hardware type
    public void imuTelemetry() {
        for (int i = 0; i < count(HardwareType.IMU); i++) {
            String name = deviceMap.get(HardwareType.IMU).get(i);
            IMU imu = getHardware(name, IMU.class);
            opMode.telemetry.addData(name + " Angles", imu.getRobotYawPitchRollAngles());
            opMode.telemetry.addData(name + " Type", imu.getClass().getCanonicalName());
            opMode.telemetry.addData(name + " Recommendation ", recommendIMU(imu));
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
          //  opMode.telemetry.addLine(recommendMotor(motor));
            opMode.telemetry.addLine();
        }
        opMode.telemetry.update();
    }
    public void encoderTelemetry(){
        for (int i = 0; i < count(HardwareType.MOTOR); i++) {
            String name = deviceMap.get(HardwareType.MOTOR).get(i);
            DcMotor motor = getHardware(name, DcMotor.class);
            opMode.telemetry.addData(name + " Position", motor.getCurrentPosition());
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
