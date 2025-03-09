package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Config.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.gentrifiedApps.gentrifiedAppsUtil.config.ConfigMaker;
import org.gentrifiedApps.gentrifiedAppsUtil.config.ConfigCreator;

public final class ConfigRegistrar {

    static ConfigMaker config = new ConfigMaker("generated");
    static int motorCount = 0;
    static int servoCount = 0;
    static int i2cCount = 0;
    static int digitalDeviceCount = 0;
    static int analogDeviceCount = 0;

    static {
        hardwareMap.forEach((name, type) -> {
            if (type == HardwareType.MOTOR){
                if (motorCount < 4) {
                    config = config.addMotor(name, ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.MotorType.RevRoboticsUltraplanetaryHDHexMotor,motorCount);
                }else {
                    config = config.addMotor(name, ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.MotorType.RevRoboticsUltraplanetaryHDHexMotor,motorCount);
                }
                motorCount += 1;
            }else if (type == HardwareType.SERVO){
                if (servoCount < 6) {
                    config = config.addDevice(name, ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.Servo,servoCount);
                }else {
                    config = config.addDevice(name, ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.DeviceType.Servo,servoCount);
                }
                servoCount += 1;
            }else if (type == HardwareType.BLINKIN){
                if (servoCount < 6) {
                    config = config.addDevice(name, ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.RevBlinkinLedDriver,servoCount);
                }else {
                    config = config.addDevice(name, ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.DeviceType.RevBlinkinLedDriver,servoCount);
                }
            }else if (type == HardwareType.TOUCH_SENSOR){
                if (digitalDeviceCount < 8) {
                    config = config.addDevice(name, ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.DigitalDevice,digitalDeviceCount);
                }else {
                    config = config.addDevice(name, ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.DeviceType.DigitalDevice,digitalDeviceCount);
                }
                digitalDeviceCount += 1;
            }else if (type == HardwareType.POTENTIOMETER){
                if (analogDeviceCount < 4) {
                    config = config.addDevice(name, ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.AnalogInput,analogDeviceCount);
                }else {
                    config = config.addDevice(name, ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.DeviceType.AnalogInput,analogDeviceCount);
                }
                analogDeviceCount += 1;
            }else if (type == HardwareType.COLOR_SENSOR){
                if (i2cCount < 4) {
                    config = config.addDevice(name, ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.RevColorSensorV3,i2cCount);
                }else {
                    config = config.addDevice(name, ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.DeviceType.RevColorSensorV3,i2cCount);
                }
                i2cCount += 1;
            }else if (type == HardwareType.DISTANCE_SENSOR){
                // cant add distance sensor
            }else if (type == HardwareType.MAGNETIC_LIMIT_SWITCH){
                if (digitalDeviceCount < 8) {
                    config = config.addDevice(name, ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.DigitalDevice,digitalDeviceCount);
                }else {
                    config = config.addDevice(name, ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.DeviceType.DigitalDevice,digitalDeviceCount);
                }
                digitalDeviceCount += 1;
            }else if (type == HardwareType.EXTERNAL_ENCODER){
                // cant add external encoder
            }
        });
    }

    static boolean isEnabled = true;
    private ConfigRegistrar() {
    }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup("xConfig")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (!isEnabled) return;
        manager.register(metaForClass(ConfigCreator.class), new ConfigCreator(config));
    }
}