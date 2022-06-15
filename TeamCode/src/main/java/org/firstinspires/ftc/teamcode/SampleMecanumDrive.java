package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class SampleMecanumDrive {
    RevBulkData bulkData;
    ExpansionHubEx expansionHub1, expansionHub2;
    public ExpansionHubMotor lf, rf, lr, rr;
    ExpansionHubMotor intake, slides, slides2, turret;

    public List<ExpansionHubMotor> motors;
    public List<Servo> servos;

    public CRServo duckSpin, duckSpin2;
    public AnalogInput leftIntake, rightIntake, depositSensor, distLeft, distRight, magLeft, magRight, flex;
    public VoltageSensor batteryVoltage;
    BNO055IMU imu;

    public Localizer localizer = new Localizer();

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initServos(hardwareMap);
        initSensors(hardwareMap);
    }

    public void initMotors (HardwareMap hardwareMap) {
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        lf = (ExpansionHubMotor) hardwareMap.dcMotor.get("lf"); //name in string is the name in robot config
        rf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");
        lr = (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rr = (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        motors.get(1).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get(2).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get(3).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.get(4).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motors = Arrays.asList(lf, lr, rr, rf, intake, turret, slides, slides2);
    }

    public void setDriveMode(DcMotor.RunMode runMode) {
        lf.setMode(runMode);
        rf.setMode(runMode);
        lr.setMode(runMode);
        rr.setMode(runMode);
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

    public void drive(Gamepad gamepad) {
        double forward = gamepad1.right_stick_y * -1;
        double left =  gamepad1.left_stick_x * 0.6;
        double turn = gamepad1.right_stick_x * 0.5;

        double p1 = forward + left + turn;
        double p2 = forward - left + turn;
        double p3 = forward + left - turn;
        double p4 = forward - left - turn;

        motors.get(0).setPower(p1);
        motors.get(1).setPower(p4);
        motors.get(2).setPower(p2);
        motors.get(3).setPower(p3);
    }
}
