package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Simulator;

public class CameraSim extends Simulator {
    LinearOpMode opMode;
    ElapsedTime runtime = new ElapsedTime();
    Robot robot;

    public CameraSim(LinearOpMode opMode) {
        runtime.reset();
        this.opMode = opMode;
        robot = new Robot(opMode);
    }

    @Override
    public void init() {
        robot.init(this);
        telemetryInit();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void telemetryInit() {
        opMode.telemetry.addLine("Plug in cameras according to configuration, then check camera stream by pressing three dots and Camera Stream.");
        opMode.telemetry.update();
    }

    @Override
    public void run() {

    }

    @Override
    public void telemetry() {

    }
}
