package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Config.hardwareMap;
import static org.firstinspires.ftc.teamcode.util.Util.approve;
import static org.firstinspires.ftc.teamcode.util.Util.blankArray;

import android.util.Pair;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;

public class Robot {
    private LinearOpMode opMode = null;
    private final Map<HardwareType, Pair<Integer, ArrayList>> hardMap = new EnumMap<>(HardwareType.class);
    {
        for (HardwareType type : HardwareType.values()) {
            hardMap.put(type, new Pair<>(type.ordinal(), new ArrayList<>()));
        }
    }

    private RevBlinkinLedDriver.BlinkinPattern blinkinPattern;

    private final double[] indexes = blankArray(HardwareType.values().length);

    public Robot(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    public void init() {
        approve(opMode);

        for (Map.Entry<HardwareType, String> entry : hardwareMap.entrySet()) {
            HardwareType hardwareType = entry.getKey();
            String hardwareName = entry.getValue();

            hardMap.get(hardwareType).second.add(opMode.hardwareMap.get(Util.getClass(hardwareType), hardwareName));
            indexes[hardMap.get(hardwareType).first]++;
        }
    }

    public void setPower(double power) {
        for (DcMotor motor : (ArrayList<DcMotor>) hardMap.get(HardwareType.MOTOR).second) {
            motor.setPower(power);
        }
    }

    public void motorTelemetry() {
        for (int i = 0; i < indexes[hardMap.get(HardwareType.MOTOR).first]; i++) {
            opMode.telemetry.addData("Motor " + i + " Power", ((DcMotor) hardMap.get(HardwareType.MOTOR).second.get(i)).getPower());
            opMode.telemetry.update();
        }
    }

    public void blinkTelemetry() {
        for (int i = 0; i < indexes[hardMap.get(HardwareType.BLINKIN).first]; i++) {
            opMode.telemetry.addData("Blinkin " + i + " Pattern", blinkinPattern);
            opMode.telemetry.update();
        }
    }

    public void servoTelemetry() {
        for (int i = 0; i < indexes[hardMap.get(HardwareType.SERVO).first]; i++) {
            opMode.telemetry.addData("Servo " + i + " Position", ((Servo) hardMap.get(HardwareType.SERVO).second.get(i)).getPosition());
            opMode.telemetry.update();
        }
    }

    public void setPosition(double position) {
        for (Servo servo : (ArrayList<Servo>) hardMap.get(HardwareType.SERVO).second) {
            servo.setPosition(position);
        }
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinPattern = pattern;
        for (RevBlinkinLedDriver blinkin : (ArrayList<RevBlinkinLedDriver>) hardMap.get(HardwareType.BLINKIN).second) {
            blinkin.setPattern(blinkinPattern);
        }
    }

    public void colorTelemetry() {
        for (int i = 0; i < indexes[hardMap.get(HardwareType.COLOR_SENSOR).first]; i++) {
            opMode.telemetry.addData("Color " + i + " Red", ((com.qualcomm.robotcore.hardware.ColorSensor) hardMap.get(HardwareType.COLOR_SENSOR).second.get(i)).red());
            opMode.telemetry.addData("Color " + i + " Green", ((com.qualcomm.robotcore.hardware.ColorSensor) hardMap.get(HardwareType.COLOR_SENSOR).second.get(i)).green());
            opMode.telemetry.addData("Color " + i + " Blue", ((com.qualcomm.robotcore.hardware.ColorSensor) hardMap.get(HardwareType.COLOR_SENSOR).second.get(i)).blue());
            opMode.telemetry.update();
        }
    }
    public void distanceTelemetry() {
        for (int i = 0; i < indexes[hardMap.get(HardwareType.DISTANCE_SENSOR).first]; i++) {
            opMode.telemetry.addData("Distance (cm) " + i + " Distance", ((com.qualcomm.robotcore.hardware.DistanceSensor) hardMap.get(HardwareType.DISTANCE_SENSOR).second.get(i)).getDistance(DistanceUnit.CM));
            opMode.telemetry.update();
        }
    }
    public void potentTelemetry() {
        for (int i = 0; i < indexes[hardMap.get(HardwareType.POTENTIOMETER).first]; i++) {
            opMode.telemetry.addData("Potentiometer " + i + " Voltage", ((com.qualcomm.robotcore.hardware.AnalogInput) hardMap.get(HardwareType.POTENTIOMETER).second.get(i)).getVoltage());
            opMode.telemetry.update();
        }
    }
    public void touchTelemetry() {
        for (int i = 0; i < indexes[hardMap.get(HardwareType.TOUCH_SENSOR).first]; i++) {
            opMode.telemetry.addData("Touch " + i + " Pressed", ((com.qualcomm.robotcore.hardware.TouchSensor) hardMap.get(HardwareType.TOUCH_SENSOR).second.get(i)).isPressed());
            opMode.telemetry.update();
        }
    }
    public void magnetTelemetry() {
        for (int i = 0; i < indexes[hardMap.get(HardwareType.MAGNETIC_LIMIT_SWITCH).first]; i++) {
            opMode.telemetry.addData("Limit " + i + " Pressed", ((com.qualcomm.robotcore.hardware.DigitalChannel) hardMap.get(HardwareType.MAGNETIC_LIMIT_SWITCH).second.get(i)).getState());
            opMode.telemetry.update();
        }
    }

}
