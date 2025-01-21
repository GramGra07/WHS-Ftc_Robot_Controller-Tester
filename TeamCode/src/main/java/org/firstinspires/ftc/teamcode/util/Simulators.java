package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Simulator;
import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.sim.BlinkinSim;
import org.firstinspires.ftc.teamcode.sim.ColorSim;
import org.firstinspires.ftc.teamcode.sim.DistanceSim;
import org.firstinspires.ftc.teamcode.sim.MagnetSim;
import org.firstinspires.ftc.teamcode.sim.MotorSim;
import org.firstinspires.ftc.teamcode.sim.PotentSim;
import org.firstinspires.ftc.teamcode.sim.ServoSim;
import org.firstinspires.ftc.teamcode.sim.TouchSim;

import java.util.EnumMap;
import java.util.Map;

public class Simulators {
    public Simulator getSimulator(HardwareType type, LinearOpMode opMode) {
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
            default:
                throw new IllegalArgumentException("Unknown HardwareType: " + type);
        }
    }
}
