package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.Util.portMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.HardwareType;
import org.firstinspires.ftc.teamcode.enums.PortType;
import org.firstinspires.ftc.teamcode.util.exceptions.TooManyDevices;

import java.util.Map;

public class Approvals {

    // This method checks if the number of devices of each type is within the limits
    public static void approve(LinearOpMode opMode) {
        double motorLimit = 4;
        double servoLimit = 6;
        double i2cLimit = 4;
        double analogLimit = 4;
        double digitalLimit = 8;
        for (Map.Entry<HardwareType, PortType> entry : portMap.entrySet()) {
            double count = Util.count(entry.getKey());
            double limit = 0;
            double hubCount = Util.hubCount(opMode.hardwareMap);
            switch (entry.getValue()) {
                case MOTOR:
                    limit = motorLimit;
                    break;
                case SERVO:
                    limit = servoLimit;
                    break;
                case I2C:
                    limit = i2cLimit;
                    break;
                case ANALOG:
                    limit = analogLimit;
                    break;
                case DIGITAL:
                    limit = digitalLimit;
                    break;
            }
            limit *= hubCount;
            if (count > limit) {
                throw new TooManyDevices("Too many " + entry.getValue().name());
            }
        }
    }
}
