package org.firstinspires.ftc.teamcode.sim;

import static org.firstinspires.ftc.teamcode.util.Util.progressTelemetry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Simulator;

import java.util.Arrays;

public class PotentSim extends Simulator {
    LinearOpMode opMode;
    ElapsedTime runtime = new ElapsedTime();

    public PotentSim(LinearOpMode opMode) {
        runtime.reset();
        this.opMode = opMode;
        robot = new Robot(opMode);
    }

    Robot robot;

    @Override
    public void init() {
        robot.init();
        telemetryInit();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void telemetryInit() {
        opMode.telemetry.addLine("Plug in the potentiometer according to configuration, then run the opMode. It will display the angle sensed by the sensor.");
        opMode.telemetry.update();
    }


    @Override
    public void run() {
    }

    @Override
    public void telemetry() {
        robot.potentTelemetry();
    }
}
