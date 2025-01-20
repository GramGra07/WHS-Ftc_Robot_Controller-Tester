package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Config.hardwareMap;
import static org.firstinspires.ftc.teamcode.util.Util.blankArray;
import static org.firstinspires.ftc.teamcode.util.Verifications.approveMotor;
import static org.firstinspires.ftc.teamcode.util.Verifications.approveServo;

import android.util.Pair;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.ArrayList;
import java.util.Map;

public class Robot {
    private LinearOpMode opMode = null;

    //TODO: If you have other ones, add them here
    private final Map<HardwareType, Pair<Integer, ArrayList>> hardMap = Map.of(
            HardwareType.values()[0], new Pair(0, new ArrayList<>()),
            HardwareType.values()[1], new Pair(1, new ArrayList<>()),
            HardwareType.values()[2], new Pair(2, new ArrayList<>())
    );
    private double motorPower;
    private double servoPosition;
    private RevBlinkinLedDriver.BlinkinPattern blinkinPattern;

    private final double[] indexes = blankArray(HardwareType.values().length);

    public Robot(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    public void init() {
        double hubCount = Util.hubCount(opMode.hardwareMap);

        //TODO: If you have other ones, add them here
        double motorCount = Util.count(HardwareType.MOTOR);
        double servoCount = Util.count(HardwareType.SERVO);
        double blinkinCount = Util.count(HardwareType.BLINKIN);

        approveMotor(motorCount, hubCount);
        approveServo(servoCount + blinkinCount, hubCount);


        for (Map.Entry<HardwareType, String> entry : hardwareMap.entrySet()) {
            HardwareType hardwareType = entry.getKey();
            String hardwareName = entry.getValue();
            switch (hardwareType) {
                case MOTOR:
                    hardMap.get(HardwareType.MOTOR).second.add(opMode.hardwareMap.get(DcMotor.class, hardwareName));
                    indexes[hardMap.get(HardwareType.MOTOR).first]++;
                    break;
                case SERVO:
                    hardMap.get(HardwareType.SERVO).second.add(opMode.hardwareMap.get(Servo.class, hardwareName));
                    indexes[hardMap.get(HardwareType.SERVO).first]++;
                    break;
                case BLINKIN:
                    hardMap.get(HardwareType.BLINKIN).second.add(opMode.hardwareMap.get(RevBlinkinLedDriver.class, hardwareName));
                    indexes[hardMap.get(HardwareType.BLINKIN).first]++;
                    break;
            }
        }
        motorPower = 0;
        servoPosition = 0;
    }

    public void setPower(double power) {
        motorPower = power;
        for (DcMotor motor : (ArrayList<DcMotor>) hardMap.get(HardwareType.MOTOR).second) {
            motor.setPower(motorPower);
        }
    }

    public void motorTelemetry() {
        for (int i = 0; i < indexes[0]; i++) {
            opMode.telemetry.addData("Motor " + i + " Power", ((DcMotor) hardMap.get(HardwareType.MOTOR).second.get(i)).getPower());
            opMode.telemetry.update();
        }
    }

    public void blinkTelemetry() {
        for (int i = 0; i < indexes[2]; i++) {
            opMode.telemetry.addData("Blinkin " + i + " Pattern", blinkinPattern);
            opMode.telemetry.update();
        }
    }

    public void servoTelemetry() {
        for (int i = 0; i < indexes[1]; i++) {
            opMode.telemetry.addData("Servo " + i + " Position", ((Servo) hardMap.get(HardwareType.SERVO).second.get(i)).getPosition());
            opMode.telemetry.update();
        }
    }

    public void setPosition(double position) {
        servoPosition = position;
        for (Servo servo : (ArrayList<Servo>) hardMap.get(HardwareType.SERVO).second) {
            servo.setPosition(servoPosition);
        }
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinPattern = pattern;
        for (RevBlinkinLedDriver blinkin : (ArrayList<RevBlinkinLedDriver>) hardMap.get(HardwareType.BLINKIN).second) {
            blinkin.setPattern(blinkinPattern);
        }
    }
}
