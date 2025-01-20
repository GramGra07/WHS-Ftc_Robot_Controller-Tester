package org.firstinspires.ftc.teamcode.util;

public class Verifications {
    public static Exception approveMotor(double motorCount, double hubCount) {
        if (motorCount > 4 * hubCount) {
            return new IllegalArgumentException("Too many motors");
        }
        return null;
    }

    public static Exception approveServo(double servoCount, double hubCount) {
        if (servoCount > 6 * hubCount) {
            return new IllegalArgumentException("Too many servos or blinkins");
        }
        return null;
    }
}
