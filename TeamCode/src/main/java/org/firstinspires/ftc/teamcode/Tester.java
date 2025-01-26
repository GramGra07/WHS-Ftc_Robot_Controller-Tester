package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.util.Util;

public class Tester extends LinearOpMode {
    Simulator simulator;

    public Tester(HardwareType type) {
        simulator = Util.getSimulator(type, this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        simulator.init();
        waitForStart();
        simulator.start();
        while (opModeIsActive()) simulator.run();
    }
}
