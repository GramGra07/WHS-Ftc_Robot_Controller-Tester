package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.sim.BlinkinSim;
import org.firstinspires.ftc.teamcode.sim.MotorSim;
import org.firstinspires.ftc.teamcode.sim.ServoSim;

public class Tester extends LinearOpMode {
    Simulator simulator;

    //TODO: If you have other ones, add them here
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
