package org.firstinspires.ftc.teamcode.util;

public class StringFuncs {
    public static String camelCase(String str) {
        return str.substring(0, 1).toUpperCase() + str.substring(1).toLowerCase();
    }
}
