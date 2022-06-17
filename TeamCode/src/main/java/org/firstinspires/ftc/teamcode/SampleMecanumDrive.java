package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.renderscript.RenderScript;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EE_Ctrl;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class SampleMecanumDrive {
    FtcDashboard dashboard;

    RevBulkData bulkData;
    ExpansionHubEx expansionHub1, expansionHub2;
    public ExpansionHubMotor lf, rf, lr, rr;
    ExpansionHubMotor intake, slides, slides2, turret;

    public List<ExpansionHubMotor> motors;
    public List<Servo> servos;

    public CRServo duckSpin, duckSpin2;
    public AnalogInput leftIntake, rightIntake, depositSensor, distLeft, distRight, magLeft, magRight, flex;
    public VoltageSensor batteryVoltage;
    public BNO055IMU imu;

    public Localizer localizer = new Localizer();

    public int[] encoders = new int[3];

    public ArrayList<UpdatePriority> motorPriorities = new ArrayList<>();

    int loops = 0;
    long start = 0;

    public boolean updateHub2 = false;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initServos(hardwareMap);
        initSensors(hardwareMap);

        localizer.imu = imu;

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25); // sends packets every 25 milliseconds
    }

    public void initMotors (HardwareMap hardwareMap) {
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        lf = (ExpansionHubMotor) hardwareMap.dcMotor.get("lf"); //name in string is the name in robot config
        rf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");
        lr = (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rr = (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        intake = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");
        slides = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides");
        slides2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides2");
        turret = (ExpansionHubMotor) hardwareMap.dcMotor.get("turret");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = Arrays.asList(lf, lr, rr, rf, intake, turret, slides, slides2);

        for(int i = 0; i < 4; i++) {
            motorPriorities.add(new UpdatePriority(3, 5));
        }
        motorPriorities.add(new UpdatePriority(1, 2)); // intake
        motorPriorities.add(new UpdatePriority(1, 3)); // turret
        motorPriorities.add(new UpdatePriority(2, 6)); // slides
    }

    public void setDriveMode(DcMotor.RunMode runMode) {
        lf.setMode(runMode);
        rf.setMode(runMode);
        lr.setMode(runMode);
        rr.setMode(runMode);
    }

    public void setMotorPowers(double[] powers) {
        for (int i = 0; i < 4; i ++){
            motorPriorities.get(i).setTargetPower(powers[i]);
        }
    }

    public void initServos(HardwareMap hardwareMap) {
        servos = new ArrayList<>();

        servos.add(hardwareMap.servo.get("rightIntake"));
        servos.add(hardwareMap.servo.get("leftIntake"));
        servos.add(hardwareMap.servo.get("deposit"));
        servos.add(hardwareMap.servo.get("odoLift"));
        servos.add(hardwareMap.servo.get("v4bar"));
        servos.add(hardwareMap.servo.get("rightCapstone"));
        servos.add(hardwareMap.servo.get("leftCapstone"));
        servos.add(hardwareMap.servo.get("duckSpinSpin"));
        servos.add(hardwareMap.servo.get("rightOdo"));
        servos.add(hardwareMap.servo.get("leftOdo"));

        duckSpin = hardwareMap.crservo.get("duckSpin");
        duckSpin = hardwareMap.crservo.get("duckSpin2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    private void initSensors(HardwareMap hardwareMap) {
        leftIntake = hardwareMap.analogInput.get("leftIntake");
        rightIntake = hardwareMap.analogInput.get("rightIntake");
        depositSensor = hardwareMap.analogInput.get("depositSensor");
        distLeft = hardwareMap.analogInput.get("distLeft");
        distRight = hardwareMap.analogInput.get("distRight");
        magLeft = hardwareMap.analogInput.get("magLeft");
        magRight = hardwareMap.analogInput.get("magRight");
        flex = hardwareMap.analogInput.get("flex");

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        for(LynxModule lynxModule: hardwareMap.getAll(LynxModule.class)) {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void drive(double forward, double left, double turn) {
        double p1 = forward + left + turn;
        double p2 = forward - left + turn;
        double p3 = forward + left - turn;
        double p4 = forward - left - turn;

        double powers[] = {p1, p2, p3, p4};
        setMotorPowers(powers);
    }

    public void update() {
        double loopStart = System.nanoTime();

        if (loops == 0) {
            start = System.currentTimeMillis();
        }
        loops++;

        getEncoders();

        double loopTime = (System.nanoTime() - loopStart) / (double) (1e9);
        double targetLoopLength = 0.01;
        double bestMotorUpdate = 1;

        int numMotorUpdated = 0;

        while((bestMotorUpdate > 0) && (loopTime <= targetLoopLength)) {
            int bestIndex = 0;
            bestMotorUpdate = motorPriorities.get(0).getPriority();
            for (int i = 1; i < motorPriorities.size(); i++) {
                if (motorPriorities.get(i).getPriority() > bestMotorUpdate) {
                    bestIndex = i;
                    bestMotorUpdate = motorPriorities.get(i).getPriority();
                }
            }
            if(bestMotorUpdate != 0) {
                numMotorUpdated++;
                motors.get(bestIndex).setPower(motorPriorities.get(bestIndex).power);
                if(bestIndex == motorPriorities.size() - 1) { //size  - 1 = the last index in the motors array
                    numMotorUpdated++;
                    slides2.setPower(motorPriorities.get(bestIndex).power);
                }
                motorPriorities.get(bestIndex).update();
            }
            loopTime = (System.nanoTime() - loopStart) / (double) (1e9);
        }


        TelemetryPacket telemetryPacket = new TelemetryPacket();
        Canvas fieldOverlay = telemetryPacket.fieldOverlay();
        fieldOverlay.strokeCircle(localizer.x, localizer.y,9);
        fieldOverlay.strokeLine(localizer.x, localizer.y, localizer.x + 11 * Math.cos(localizer.heading), localizer.y + 11 * Math.sin(localizer.heading));

        fieldOverlay.strokeCircle(localizer.leftSensor.x, localizer.leftSensor.y, 2).setStroke("#FF0000");
        fieldOverlay.strokeCircle(localizer.rightSensor.x, localizer.rightSensor.y, 2).setStroke("#FF0000");

        dashboard.sendTelemetryPacket(telemetryPacket);
        telemetryPacket.put("Loop Speed: ", (double) (System.currentTimeMillis() - start) / (loops));
        telemetryPacket.put("Person: ", "Alex");
        telemetryPacket.put("num Motors Updated: ", numMotorUpdated);

        updateHub2 = false;
        updateHub2();
    }

    public double currentIntakeSpeed;
    public int rightIntakeVal;
    public int leftIntakeVal;
    public int depositVal;
    public int flexSensorVal;
    public double currentSlideLength, currentSlideSpeed;
    public double currentTurretAngle;
    public double distValLeft, distValRight;
    public int magValLeft, magValRight;
    public double lastDistValRight = 0;
    public double lastDistValLeft = 0;

    public void updateHub2() {
        if(!updateHub2) {
            updateHub2 = true;
            bulkData = expansionHub2.getBulkInputData();
            if (bulkData != null) {
                try {
                    currentSlideLength = bulkData.getMotorCurrentPosition(slides2) / 25.1372713591; //gear ratio * circumference of spool / ticks per revolution
                    currentSlideSpeed = bulkData.getMotorVelocity(slides2) / 25.1372713591; // 25.1372713591 is the amount of ticks per revolution
                    currentTurretAngle = bulkData.getMotorCurrentPosition(turret) / 578.3213; // conversion factor to radians
                    distValLeft = bulkData.getAnalogInputValue(distLeft) / 3.2; // analog sensor returns value as a voltage --> analog distance sensor convert volts to inches. 3.2 is conversion factor from volts to inches
                    distValRight = bulkData.getAnalogInputValue(distRight) / 3.2;
                    magValLeft = (bulkData.getAnalogInputValue(magLeft));
                    magValRight = (bulkData.getAnalogInputValue(magRight));

                    if (lastDistValLeft != distValLeft || lastDistValRight != distValRight) {
                        localizer.distUpdate(distValRight,distValLeft);
                    }
                    lastDistValLeft = distValLeft;
                    lastDistValRight = distValRight;

                } catch(Exception e) {
                    e.printStackTrace();
                    Log.e("****EXCEPTION: ", e.getClass().getName());
                }
            }
        }
    }

    public void getEncoders() {
        bulkData = expansionHub1.getBulkInputData();
        if (bulkData != null) {
            try {
                encoders[0] = bulkData.getMotorCurrentPosition(rf);
                encoders[1] = bulkData.getMotorCurrentPosition(lf);
                encoders[2] = bulkData.getMotorCurrentPosition(rr);

                localizer.updateEncoders(encoders);

                flexSensorVal = (bulkData.getAnalogInputValue(flex));
                localizer.updateFlex(flexSensorVal);

                localizer.update();
            }
            catch(Exception e) {
                e.printStackTrace();
                Log.e("****EXCEPTION: ", e.getClass().getName());
            }
        }
    }
}
