package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.sim.BlinkinSim;
import org.firstinspires.ftc.teamcode.sim.ColorSim;
import org.firstinspires.ftc.teamcode.sim.DistanceSim;
import org.firstinspires.ftc.teamcode.sim.MagnetSim;
import org.firstinspires.ftc.teamcode.sim.MotorSim;
import org.firstinspires.ftc.teamcode.sim.PotentSim;
import org.firstinspires.ftc.teamcode.sim.ServoSim;
import org.firstinspires.ftc.teamcode.sim.TouchSim;

public class Tester extends LinearOpMode {
    Simulator simulator;

    public Tester(HardwareType type) {
        switch (type) {
            case MOTOR:
                simulator = new MotorSim(this);
                break;
            case SERVO:
                simulator = new ServoSim(this);
                break;
            case BLINKIN:
                simulator = new BlinkinSim(this);
                break;
            case TOUCH_SENSOR:
                simulator = new TouchSim(this);
                break;
            case POTENTIOMETER:
                simulator = new PotentSim(this);
                break;
            case COLOR_SENSOR:
                simulator = new ColorSim(this);
                break;
            case DISTANCE_SENSOR:
                simulator = new DistanceSim(this);
                break;
            case MAGNETIC_LIMIT_SWITCH:
                simulator = new MagnetSim(this);
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        simulator.init();
        waitForStart();
        simulator.start();
        while (opModeIsActive()) simulator.run();
    }
}
