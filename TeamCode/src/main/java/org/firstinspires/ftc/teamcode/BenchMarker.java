package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.Alliance;
import org.gentrifiedApps.gentrifiedAppsUtil.dataStorage.DataStorage;

import java.sql.Time;
import java.util.ArrayList;
import java.util.List;

@TeleOp
public class BenchMarker extends LinearOpMode {
    Timer startTimer = new Timer("start");


    class Timer {
        String name;
        public Timer(String name){
            this.name = name;
        }
        double start = System.currentTimeMillis();
        double get(){
            return System.currentTimeMillis() - start;
        }
        void log(){
            Scribe.getInstance().logData(name + this.get() + "ms");
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        RevColorSensorV3 c1 = hardwareMap.get(RevColorSensorV3.class, "testi2c");
        RevColorSensorV3 c2 = hardwareMap.get(RevColorSensorV3.class, "testi2cFAST");


        startTimer.log();
        waitForStart();
        Timer motorState = new Timer("Switch Motor State to Reset -> Use Encoder");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorState.log();
        Timer motorPower = new Timer ("Set 1-> 0");
        motor.setPower (1);
        motor.setPower (0);
        motorPower.log();
        Timer listDoubles = new Timer ("A lot of doubles (arr)");
        int lot = 1000000;
        double[] doubles = new double[lot];
        for(int i = 0; i < lot; i++){
            doubles[i] = Math.random();
        }
        listDoubles.log();
        Timer list2Doubles = new Timer("A lot of doubles (mutableList)");
        List<Double> mutableList = new ArrayList<>();
        for(int i = 0; i < lot; i++){
            mutableList.add(Math.random());
        }
        list2Doubles.log();
        Timer dataStore = new Timer ("Data storage");
        DataStorage.setAlliance(Alliance.BLUE);
        DataStorage.getAlliance();
        dataStore.log();
        Timer blacksboard = new Timer("Blackboard");
        blackboard.put("allinace",0);
        blackboard.get("allinace");
        blacksboard.log();
    }
}
