package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Simulator;

import java.util.Arrays;

public class ServoSim extends Simulator {
    LinearOpMode opMode;
    Robot robot;
    ElapsedTime runtime = new ElapsedTime();

    public ServoSim(LinearOpMode myOpMode) {
        runtime.reset();
        this.opMode = myOpMode;
        robot = new Robot(myOpMode);
    }

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
        opMode.telemetry.addLine("Plug in servos according to configuration, then run the opMode. It will set position to " + Arrays.toString(Config.servoPosition));
        opMode.telemetry.update();
    }

    @Override
    public void run() {
        double[] positions = Config.servoPosition;
        for (double position : positions) {
            while (runtime.seconds() < Config.delay && !opMode.isStopRequested()) {
                robot.setPosition(position);
                telemetry();
            }
            runtime.reset();
        }
    }

    @Override
    public void telemetry() {
        robot.servoTelemetry();
    }
}
