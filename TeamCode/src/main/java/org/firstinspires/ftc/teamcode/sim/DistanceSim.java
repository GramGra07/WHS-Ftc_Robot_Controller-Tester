package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Simulator;

public class DistanceSim extends Simulator {
    LinearOpMode opMode;
    ElapsedTime runtime = new ElapsedTime();
    Robot robot;

    public DistanceSim(LinearOpMode opMode) {
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
        opMode.telemetry.addLine("Plug in the distance sensor according to configuration, then run the opMode. It will display the distance sensed by the sensor.");
        opMode.telemetry.update();
    }

    @Override
    public void run() {
        telemetry();
    }

    @Override
    public void telemetry() {
        robot.distanceTelemetry();
    }
}
