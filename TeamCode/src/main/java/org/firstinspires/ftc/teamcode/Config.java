package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.enums.HardwareType;

import java.util.HashMap;
import java.util.Map;

public final class Config {
    public static Map<String, HardwareType> hardwareMap = new HashMap<>();
    public static double delay = 1;
    public static double[] servoPosition = {0, 1, 0.5, 0};
    public static RevBlinkinLedDriver.BlinkinPattern[] blinkinPattern = {RevBlinkinLedDriver.BlinkinPattern.BLACK, RevBlinkinLedDriver.BlinkinPattern.BLUE, RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE, RevBlinkinLedDriver.BlinkinPattern.BREATH_RED, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE,};

    //TODO: If you have other ones, add them here
    static {
        hardwareMap.put("motor1", HardwareType.MOTOR);
        hardwareMap.put("motor2", HardwareType.MOTOR);
        hardwareMap.put("motor3", HardwareType.MOTOR);
        hardwareMap.put("motor4", HardwareType.MOTOR);
        hardwareMap.put("servo1", HardwareType.SERVO);
        hardwareMap.put("servo2", HardwareType.SERVO);
        hardwareMap.put("servo3", HardwareType.SERVO);
        hardwareMap.put("servo4", HardwareType.SERVO);
        hardwareMap.put("servo5", HardwareType.SERVO);
        hardwareMap.put("servo6", HardwareType.SERVO);
        hardwareMap.put("Webcam 1", HardwareType.CAMERA);

      //  hardwareMap.put("blink", HardwareType.BLINKIN);
      //  hardwareMap.put("imu", HardwareType.IMU);
      //  hardwareMap.put("imu2", HardwareType.IMU);
    }
}
