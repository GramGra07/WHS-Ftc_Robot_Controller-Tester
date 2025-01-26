package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Simulator;

import java.util.Arrays;

public class MotorSim extends Simulator {
    LinearOpMode opMode;
    ElapsedTime runtime = new ElapsedTime();
    Robot robot;

    public MotorSim(LinearOpMode opMode) {
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
        opMode.telemetry.addLine("Plug in motors according to configuration, then run the opMode. It will set power to " + Arrays.toString(Config.motorPower));
        opMode.telemetry.update();
    }

    @Override
    public void run() {
        double[] powers = Config.motorPower;
        for (double power : powers) {
            while (runtime.seconds() < Config.delay && !opMode.isStopRequested()) {
                robot.setPower(power);
                telemetry();
            }
            runtime.reset();
        }
    }

    @Override
    public void telemetry() {
        robot.motorTelemetry();
    }
}
