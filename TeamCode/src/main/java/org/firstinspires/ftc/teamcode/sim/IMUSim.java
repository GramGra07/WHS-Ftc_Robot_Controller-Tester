package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Simulator;

public class IMUSim extends Simulator {
    LinearOpMode opMode;
    ElapsedTime runtime = new ElapsedTime();
    Robot robot;

    public IMUSim(LinearOpMode opMode) {
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
        opMode.telemetry.addLine("This will show you angles returned and the type of IMU it is.");
        opMode.telemetry.update();
    }

    @Override
    public void run() {
        telemetry();
    }

    @Override
    public void telemetry() {
        robot.imuTelemetry();
    }

}
