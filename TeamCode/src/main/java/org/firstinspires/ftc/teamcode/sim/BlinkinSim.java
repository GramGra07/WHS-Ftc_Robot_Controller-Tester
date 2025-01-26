package org.firstinspires.ftc.teamcode.sim;

import static org.firstinspires.ftc.teamcode.util.Util.progressTelemetry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Simulator;

import java.util.Arrays;

public class BlinkinSim extends Simulator {
    LinearOpMode opMode;
    ElapsedTime runtime = new ElapsedTime();
    Robot robot;
    double index = 0;

    public BlinkinSim(LinearOpMode opMode) {
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
        opMode.telemetry.addLine("Plug in blinkins according to configuration, then run the opMode. It will set pattern to " + Arrays.toString(Config.blinkinPattern));
        opMode.telemetry.update();
    }

    @Override
    public void run() {
        RevBlinkinLedDriver.BlinkinPattern[] blinks = Config.blinkinPattern;
        for (RevBlinkinLedDriver.BlinkinPattern blink : blinks) {
            index++;
            while (runtime.seconds() < Config.delay && !opMode.isStopRequested()) {
                robot.setPattern(blink);
                telemetry();
            }
            runtime.reset();
        }
    }

    @Override
    public void telemetry() {
        progressTelemetry(opMode.telemetry, Config.blinkinPattern.length, index);
        robot.blinkTelemetry();
    }
}
