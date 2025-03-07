package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.enums.HardwareType;

import java.util.HashMap;
import java.util.Map;

public final class Config {
    //TODO: If you have other ones, add them here
    public static Map<String, HardwareType> hardwareMap = new HashMap<>();
    public static double delay = 4;
    public static double[] motorPower = {0, -1, 1, 0};
    public static double[] servoPosition = {0, -1, 0, 1, 0};
    public static RevBlinkinLedDriver.BlinkinPattern[] blinkinPattern = {RevBlinkinLedDriver.BlinkinPattern.BLACK, RevBlinkinLedDriver.BlinkinPattern.BLUE, RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE, RevBlinkinLedDriver.BlinkinPattern.BREATH_RED, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE,};

    // Add your hardware here
    static {
        hardwareMap.put("pitchMotor", HardwareType.MOTOR);
        hardwareMap.put("pitchServo1", HardwareType.SERVO);
        hardwareMap.put("blink", HardwareType.BLINKIN);
        hardwareMap.put("imu", HardwareType.IMU);
        hardwareMap.put("imu2", HardwareType.IMU);
    }
}
