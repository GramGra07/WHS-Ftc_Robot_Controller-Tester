package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.StringFuncs.camelCase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.enums.HardwareType;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public final class Registrar {
    private static final List<Class<? extends OpMode>> opModeClasses = new ArrayList<>();

    static {
        opModeClasses.add(Tester.class);
    }

    private Registrar() {
    }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls, HardwareType type) {
        return new OpModeMeta.Builder()
                .setName(camelCase(type.name()) + cls.getSimpleName())
                .setGroup("Testers")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        for (Class<? extends OpMode> opModeClass : opModeClasses) {
            for (HardwareType type: Config.hardwareMap.values()) { // only registers that which you are using
                manager.register(
                        metaForClass(opModeClass, type),
                        createInstance(opModeClass, type)
                );
            }
        }
    }

    private static OpMode createInstance(Class<? extends OpMode> cls, HardwareType type) {
        try {
            return cls.getConstructor(HardwareType.class).newInstance(type);
        } catch (Exception e) {
            throw new RuntimeException("Failed to create instance of " + cls.getSimpleName(), e);
        }
    }
}