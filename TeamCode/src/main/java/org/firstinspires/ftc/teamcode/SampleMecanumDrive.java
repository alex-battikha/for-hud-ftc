package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.renderscript.RenderScript;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
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

@Config
public class SampleMecanumDrive{
    //any variables marked as public static will appear in dashboard
    //must add @Config before the class name
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

    public static double depositInterfaceAngle = 0.8;
    public static double v4barInterfaceAngle = 0.15;
    public double currentIntake = 0;

    public double leftIntakeDrop;
    public double leftIntakeRaise;
    public double rightIntakeDrop;
    public double rightIntakeRaise;
    public double leftIntakeMid;
    public double rightIntakeMid;

    public double intakeTurretInterfaceHeading = Math.toRadians(57.5);

    public double slideExtensionLength = 0;
    public double intakePos = 0;
    public double intakeSpeed = 0;
    public double intakeTicksPerRev = ((1.0+(46.0/11.0)) * 28.0) / (26.0/19.0);
    public double turretHeading = 0;
    public double targetSlideExtensionLength = 0;
    public double targetTurretHeading = 0;
    public double targetV4barOrientation = 0;
    public double slideTickToInch = 25.1372713591;
    public double turretTickToRadians = 578.3213;
    public double currentSlidesSpeed = 0;
    static double currentV4barAngle = 0;
    double targetV4barAngle = 0;

    public int dropIntakeTime = 380;
    public double intakePower = -1;
    public int liftIntakeTime = 700;
    public int transfer1Time = 215; //300
    public int transfer2Time = 235; //350
    public double transfer1Power = 1.0;
    //public double transfer2Power = 0.78;
    public int openDepositTime = 250; //400
    public int intakeLiftDelay = 100;
    public int effectiveDepositTime = openDepositTime;
    public double returnSlideLength = 0.35;

    double rightIntakeVal, leftIntakeVal, depositVal, sumIntakeSensor, intakeSensorLoops;

    long transferTime = System.currentTimeMillis();

    int numRightIntake = 0;
    int numLeftIntake = 0;

    boolean firstSlide = false;

    double targetSlidesPose = 0, slidesSpeed = 0, targetTurretPose = 0, turretPower = 0;

    double currentDepositAngle = depositInterfaceAngle;

    double targetDepositAngle = 0;

    public int slidesCase, lastSlidesCase;

    public long intakeTime, slideTime;

    long slideStart = System.currentTimeMillis();

    public double turretOffset = 0;
    public double slidesOffset = 0;
    public double v4barOffset = 0;

    public boolean deposit = false;
    public double depositTransferAngle = Math.toRadians(135);

    public double loopSpeed = 0;

    public double effectiveDepositAngle = Math.toRadians(-45);

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
    public long loopTime = 0;
    public void update() {
        double loopStart = System.nanoTime();

        if (loops == 0) {
            start = System.currentTimeMillis();
        }
        loops++;

        getEncoders();
        updateHub2();

        double loopTime = (System.nanoTime() - loopStart) / (double) (1e9);
        double targetLoopLength = 0.006;
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

        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(localizer.leftSensor.x, localizer.leftSensor.y, 2);
        fieldOverlay.strokeCircle(localizer.rightSensor.x, localizer.rightSensor.y, 2);

        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.strokeCircle(targetPoint.x, targetPoint.y, 9);

        dashboard.sendTelemetryPacket(telemetryPacket);
        telemetryPacket.put("Loop Speed: ", (double) (System.currentTimeMillis() - start) / (loops));
        telemetryPacket.put("Person: ", "Alex");
        telemetryPacket.put("num Motors Updated: ", numMotorUpdated);

        telemetryPacket.put("Heading Error: ", headingError);

        updateHub2 = false;

//        updateSlides();
//        updateIntake();
    }

    public double currentIntakeSpeed;
    public int flexSensorVal;
    public double currentSlideLength, currentSlideSpeed;
    public double currentTurretAngle;
    public double distValLeft, distValRight;
    public int magValLeft, magValRight;
    public double lastDistValRight = 0;
    public double lastDistValLeft = 0;

    public static double headingP = 1;
    public static double headingI = 0;
    public static double headingD = 0;

    double headingIntegral = 0;
    double lastHeading = 0;

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

    public void setDepositAngle(double targetAngle){
        targetDepositAngle = targetAngle;
    }

    public int lastIntakeCase = 0;
    public int intakeCase = 0; // longs are typically used for time

    public boolean transferMineral = false;
    public boolean intakeDepositTransfer = false; //outaking from the intake to the bucket

    public void updateIntake(){
        if (!transferMineral){ // mineral is not being transferred to the bucket
            setDepositAngle(depositInterfaceAngle); // these are being set to specific angles and positions to catch the mineral
            setV4barOrientation(v4barInterfaceAngle);
            setTurretTarget(intakeTurretInterfaceHeading * currentIntake);
            if (Math.abs(turretHeading) >= Math.toRadians(20)){
                setSlidesLength(returnSlideLength, 0.4);
            }
            else{
                setSlidesLength(returnSlideLength + 2.5, 0.4);
            }
            if (intakeCase == 0){
                switch ((int) currentIntake) {
                    case -1: // turret right
                        servos.get(1).setPosition(leftIntakeMid);
                        servos.get(0).setPosition(rightIntakeRaise);
                        break;
                    case 0: // turret is neutral
                        servos.get(0).setPosition(rightIntakeMid);
                        servos.get(1).setPosition(leftIntakeMid);
                        break;
                    case 1: // turret left
                        servos.get(0).setPosition(rightIntakeMid);
                        servos.get(1).setPosition(leftIntakeRaise);
                        break;
                }
                intake.setPower(0);
            }
        }
        // looped through the first time we change the intakeCase --> we want this so that we can start tracking the first time we change intake cases
        if (lastIntakeCase != intakeCase) {
            switch (intakeCase) {
                case 1: intakeSensorLoops = 1; sumIntakeSensor = 0; intake.setPower(0.3); break; // rotate the servo down
                case 2: intake.setPower(intakePower); break; // turn on the intake (forward)
                case 3: transferTime = System.currentTimeMillis();break; // lift up the servo
                case 6:
                    Log.e("liftTime" , (System.currentTimeMillis() - transferTime) + "");
                    transferTime = System.currentTimeMillis();
                    intake.setPower(transfer1Power);
                    break;
                case 7:
                    intake.setPower(transfer1Power);
                    //intake.setPower(transfer2Power);
                    break;
                case 8:
                    Log.e("transferTime" , (System.currentTimeMillis() - transferTime) + "");
                    intake.setPower(0); transferMineral = true; intakeDepositTransfer = false; // transferMineral gets set to true on the last intake case because we completed everything else for the intake
                    setDepositAngle(depositInterfaceAngle + Math.toRadians(30)); //15
                    firstSlide = false;
                    break; // turn off the intake
            }
            intakeTime = System.currentTimeMillis();
        }
        lastIntakeCase = intakeCase;
        switch (intakeCase) {
            case 1: case 2:
                if (intakeCase == 1 && System.currentTimeMillis() - intakeTime >= dropIntakeTime){intakeCase ++;}// waiting for the servo to drop
                if (intakeCase == 2 && ((currentIntake == -1 && numRightIntake >= 3) || (currentIntake == 1 && numLeftIntake >= 3)) && System.currentTimeMillis() - intakeTime >= 100){intakeCase ++;}

                if(currentIntake == 1){servos.get(1).setPosition(leftIntakeDrop);servos.get(0).setPosition(rightIntakeMid);}
                if(currentIntake == -1){servos.get(0).setPosition(rightIntakeDrop);servos.get(1).setPosition(leftIntakeMid);}
                break; // wait for block in
            case 3:
                if (System.currentTimeMillis() - intakeTime >= intakeLiftDelay) {
                    if (currentIntake == 1) {
                        servos.get(1).setPosition(leftIntakeRaise);
                    }
                    if (currentIntake == -1) {
                        servos.get(0).setPosition(rightIntakeRaise);
                    }
                }
                if ((((currentIntake == 1 && magValLeft >= 1800) || (currentIntake == -1 && magValRight >= 1800)) || System.currentTimeMillis() - intakeTime >= liftIntakeTime + intakeLiftDelay) && !transferMineral){
                    intakeCase ++;
                }
                if (!transferMineral){
                    setDepositAngle(depositInterfaceAngle);
                    setV4barOrientation(v4barInterfaceAngle);
                }
                break;  // waiting for the servo to go up && slides to be back 200 before
            //waits for turret to be in the correct orientation
            case 4: if (Math.abs(turretHeading - intakeTurretInterfaceHeading*currentIntake) <= Math.toRadians(7.5)){intakeCase ++;}break;//wait for the slides to be in the correct orientation
            //waits for slides and v4bar to be in the correct orientation
            case 5: if (Math.abs(targetV4barAngle - currentV4barAngle) < Math.toRadians(5) && Math.abs(slideExtensionLength - returnSlideLength) < 0.5){intakeCase ++;}break;
            //hard outtake
            case 6: if (System.currentTimeMillis() - intakeTime >= 200 && (intakeDepositTransfer || System.currentTimeMillis() - intakeTime >= transfer1Time)){intakeCase ++;}break; // 80
            //soft outtake
            case 7: if (System.currentTimeMillis() - intakeTime >= 30 && (intakeDepositTransfer || System.currentTimeMillis() - intakeTime >= transfer2Time)){intakeCase ++;currentDepositAngle = depositInterfaceAngle;}break;
        } //80
    }
    public void setV4barOrientation(double targetV4barOrientation){
        targetV4barAngle = targetV4barOrientation;
    }

    public void setTurretTarget(double radians){
        targetTurretPose = radians;
        turretPower = 0.35;
    }

    public void setSlidesLength(double inches){
        targetSlidesPose = inches;
        slidesSpeed = 1;
    }

    public void setSlidesLength(double inches, double speed){
        targetSlidesPose = inches;
        slidesSpeed = speed;
    }

    public void updateSlides(){
        if (transferMineral) { // I have deposited into the area
            if (lastSlidesCase != slidesCase) {
                slideTime = System.currentTimeMillis();
            }
            lastSlidesCase = slidesCase;
            int a = slidesCase;
            switch (a) {
                case 1: case 2: case 3:

                    double t = targetV4barOrientation + v4barOffset - Math.toRadians(10);

                    if (!firstSlide){
                        firstSlide = true;
                        slideStart = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - slideStart >= 80) { //30
                        setTurretTarget(targetTurretHeading + turretOffset);

                        double slidePower = 1.0;
                        double target; //depositTransferAngle
                        double speed = 0.2;
                        double l = Math.abs(slideExtensionLength - (targetSlideExtensionLength + slidesOffset));
                        if (targetSlideExtensionLength + slidesOffset <= 10) {
                            target = Math.toRadians(110);
                            slidePower = 0.5;
                        } else { //if (currentV4barAngle >= Math.toRadians(19))
                            speed = 0.6; //1.1
                            target = Math.toRadians(107.5); //127.5
                        }

                        currentDepositAngle += Math.signum(target - currentDepositAngle) * Math.min(Math.abs((depositTransferAngle - depositInterfaceAngle) / speed) * loopSpeed, Math.toRadians(1.0));
                        if (Math.abs(target - currentDepositAngle) <= Math.toRadians(1)) {
                            currentDepositAngle = target;
                        }

                        if (Math.abs(turretHeading - (targetTurretHeading + turretOffset)) <= Math.toRadians(10)) {
                            if (slidesCase == 1) {
                                setSlidesLength(4, slidePower);
                                setV4barOrientation(Math.min(Math.toRadians(137.1980907721663), t));
                            } else if (l < 10) {
                                setSlidesLength(targetSlideExtensionLength + slidesOffset, Math.max((slidePower - 0.65), 0.05) + Math.pow((targetSlideExtensionLength + slidesOffset - slideExtensionLength) / 10.0, 2) * 0.25);//o.35
                                if (targetV4barOrientation + v4barOffset >= Math.toRadians(180) && Math.abs(currentSlidesSpeed) >= 10 && Math.abs(currentV4barAngle - (targetV4barOrientation + v4barOffset - Math.toRadians(10))) >= Math.toRadians(5)) {
                                    setV4barOrientation(Math.min(Math.toRadians(137.1980907721663), t));
                                } else {
                                    setV4barOrientation(t);
                                }
                            } else {
                                setSlidesLength(targetSlideExtensionLength + slidesOffset, slidePower);
                                setV4barOrientation(Math.min(Math.toRadians(100), t));
                            }
                            setDepositAngle(currentDepositAngle);
                        }
                    }
                    else {
                        currentDepositAngle = Math.toRadians(105);
                        setDepositAngle(Math.toRadians(105));
                    }
                    /*
                    Log.e("why it fail",(Math.abs(turretHeading - (targetTurretHeading + turretOffset)) <= Math.toRadians(7.5))
                            + " " + (Math.abs(slideExtensionLength - (targetSlideExtensionLength + slidesOffset)) <= 8)
                            + " " + (Math.abs(t - currentV4barAngle) <= Math.toRadians(18))); //12
                    */
                    if (slidesCase == 1 && ((Math.abs(slideExtensionLength - 4) <= 3.5 && (currentV4barAngle >= Math.min(Math.toRadians(130),t))) || targetSlideExtensionLength + slidesOffset >= 10)){slidesCase ++;}
                    else if (slidesCase == 2 && (Math.abs(turretHeading - (targetTurretHeading + turretOffset)) <= Math.toRadians(7.5)
                            && Math.abs(slideExtensionLength - (targetSlideExtensionLength + slidesOffset)) <= 6 // 3 => 5 => 7 => 9 => 6 => 8 => 6
                            && Math.abs(t - currentV4barAngle) <= Math.toRadians(3)) //18 => 5
                    ){slidesCase ++;} // was 5 (now 7.5) || System.currentTimeMillis() - slideTime >= 400
                    if (slidesCase == 3 && deposit){slidesCase ++; currentDepositAngle += Math.toRadians(20);setDepositAngle(Math.toRadians(180) - effectiveDepositAngle);
                    updateDepositAngle();
                    }
                    break; //13 => 5 && Math.abs(currentSlidesSpeed) <= 8 &&
                case 4:
                    double depoAngle = Math.toRadians(180) - effectiveDepositAngle;
                    currentDepositAngle += Math.signum(depoAngle - currentDepositAngle) * (depoAngle - depositTransferAngle) / (0.1) * loopSpeed;
                    if (Math.abs(currentDepositAngle - depoAngle) <= Math.toRadians(2)){
                        currentDepositAngle =  depoAngle;
                    }

                    if (Math.abs(currentDepositAngle - depoAngle) <= Math.toRadians(10) && System.currentTimeMillis()-slideTime <= effectiveDepositTime - 150){
                        setV4barOrientation(targetV4barOrientation + v4barOffset);
                    }
                    else{
                        setV4barOrientation(targetV4barOrientation + v4barOffset - Math.toRadians(10));
                    }

                    setDepositAngle(depoAngle); //currentDepositAngle
                    setTurretTarget(targetTurretHeading + turretOffset);
                    setSlidesLength(targetSlideExtensionLength + slidesOffset,0.25);
                    if (slidesCase == 4 && System.currentTimeMillis() - slideTime >= effectiveDepositTime){slidesCase ++; intakeCase = 0; lastIntakeCase = 0; currentDepositAngle = depositTransferAngle;} // + 70 effectiveDepositTime
                    break;
                case 5 : case 6: case 7: case 8:
                    setDepositAngle(depositInterfaceAngle);
                    if (slidesCase == 5){
                        //DO NOTHING: NO MOVING SLIDES BACK NO MOVING V4BAR
                        if (System.currentTimeMillis() - slideTime >= 150){ //300
                            setV4barOrientation(v4barInterfaceAngle);
                        }
                    }
                    else {
                        setV4barOrientation(v4barInterfaceAngle);
                        if (currentV4barAngle >= v4barInterfaceAngle + Math.toRadians(15)){
                            setSlidesLength(returnSlideLength + 3, 0.4);
                        }
                        else{
                            setSlidesLength(returnSlideLength, 0.6);
                            if (slidesCase == 8 && Math.abs(slideExtensionLength - returnSlideLength) <= 2){
                                slidesCase ++;
                            }
                        }
                    }

                    if (slidesCase >= 7) {
                        setTurretTarget(intakeTurretInterfaceHeading * currentIntake);
                    }
                    else {
                        setTurretTarget(targetTurretHeading + turretOffset);
                    }

                    if (slidesCase == 5 && System.currentTimeMillis() - slideTime >= 250){slidesCase ++;} //400
                    else if (slidesCase == 6 && Math.abs(slideExtensionLength-returnSlideLength) <= 4){slidesCase ++;}
                    else if (slidesCase == 7 && Math.abs(turretHeading - intakeTurretInterfaceHeading*currentIntake) <= Math.toRadians(10)){slidesCase ++;}
                    break;
                case 9: //resets the slidesCase & officially says mineral has not been transferred
                    transferMineral = false; slidesCase = 0; lastSlidesCase = 0; deposit = false; effectiveDepositTime = openDepositTime;
                    break;
            }
        }
    }

    public void updateDepositAngle(){
        double angle = targetDepositAngle - currentV4barAngle;
        double targetPos = angle * 0.215820468 + 0.21;
        targetPos = Math.min(Math.max(targetPos,0.0),0.86);
        servos.get(2).setPosition(targetPos);
    }

    public Pose2D targetPoint = new Pose2D(0,0,0);

    double headingError = 0;

    public void driveToPoint(LinearOpMode opMode, Pose2D targetPoint) {
        this.targetPoint = targetPoint;
        double error = Math.sqrt(Math.pow(localizer.x - targetPoint.x, 2) + Math.pow(localizer.y - targetPoint.y, 2));
        while (opMode.opModeIsActive() && error > 3) { //more than 3 inches away from the point
            error = Math.sqrt(Math.pow(localizer.x - targetPoint.x, 2) + Math.pow(localizer.y - targetPoint.y, 2)); //distance formula
            update();
            double targetAngle = Math.atan2(targetPoint.y - localizer.y, targetPoint.x - localizer.x); //inverse tan that takes two numbers; we have the x and y but we need to find heading
            headingError = targetAngle - localizer.heading;

            while(headingError > Math.PI) {
                headingError -= Math.PI * 2; // this is for heading being out of bounds (not between -180 and 180 degree) and we want to constrict it
            }

            while(headingError < -Math.PI) {
                headingError += Math.PI*2;
            }

            headingIntegral += headingError * loopTime;

            lastHeading = headingError;


            double relErrorX = Math.cos(headingError) * error;
            double relErrorY = Math.sin(headingError) * error;

            double turn = Math.toDegrees(headingError) * 0.3 /15;//30 percent power when turning 15 degrees // t is the speed you want to turn at
            // our forward speed 0.5 is subtracted from our max speed (1.0) - turning speed
            double forward = (relErrorX / error) * targetPoint.speed * (1.0 - Math.abs(turn)); //0.5 represents the power to drive motors // this is to find the proportion between relErrorX and relErrorY
            double left = (relErrorY / error) * targetPoint.speed * (1.0 - Math.abs(turn)); //strafing

            double a1 = forward - left - turn; //in actual units turning to the right is positive but when people drive they like to turn to the right and make it positive
            double a2 = forward + left - turn;
            double a3 = forward - left + turn;
            double a4 = forward + turn + left;

            setMotorPowers(new double[]{a1, a2, a3, a4}); // pass in array directly
        }
    }

    public void followTrajectory(LinearOpMode opMode, Trajectory trajectory) {
        Pose2D targetPoint;

        while (opMode.opModeIsActive() && trajectory.points.size() != 0) { //more than 3 inches away from the point
            update();
            targetPoint = trajectory.points.get(0);
            this.targetPoint = targetPoint;

            double error = Math.sqrt(Math.pow(localizer.x - targetPoint.x, 2) + Math.pow(localizer.y - targetPoint.y, 2)); //distance formula
            double lastError = Math.sqrt(Math.pow(localizer.x - trajectory.points.get(trajectory.points.size() - 1).x, 2) + Math.pow(localizer.y - trajectory.points.get(trajectory.points.size() - 1).y, 2));

            double targetAngle = Math.atan2(targetPoint.y - localizer.y, targetPoint.x - localizer.x); //inverse tan that takes two numbers; we have the x and y but we need to find heading
            headingError = targetAngle - localizer.heading;

            double relErrorX = Math.cos(headingError) * error;
            double relErrorY = Math.sin(headingError) * error;

            double dHeadingError = (headingError - lastHeading) / loopTime;


            if(lastError < 8 && trajectory.points.size() < 100) {
                headingError = trajectory.points.get(trajectory.points.size() - 1).heading - localizer.heading;
            }

            while(headingError > Math.PI) {
                headingError -= Math.PI * 2; // this is for heading being out of bounds (not between -180 and 180 degree) and we want to constrict it
            }

            while(headingError < -Math.PI) {
                headingError += Math.PI*2;
            }

            double turn = headingError * headingP + headingIntegral * headingI + dHeadingError * headingD;
            
            // our forward speed 0.5 is subtracted from our max speed (1.0) - turning speed
            double forward = 0;
            double left = 0;
            if(error != 0) {
                forward = (relErrorX / error) * targetPoint.speed / (1.0 - Math.abs(turn)); //0.5 represents the power to drive motors // this is to find the proportion between relErrorX and relErrorY
                left = (relErrorY / error) * targetPoint.speed / (1.0 - Math.abs(turn)); //strafing
            }

            double a1 = forward - left - turn; //in actual units turning to the right is positive but when people drive they like to turn to the right and make it positive
            double a2 = forward + left - turn;
            double a3 = forward - left + turn;
            double a4 = forward + turn + left;

            setMotorPowers(new double[]{a1, a2, a3, a4}); // pass in array directly

            trajectory.update(localizer.currentPos, localizer.relativeCurrentVel);
        }
    }

}
//finite state machine (FSM) keeps track of different states for different subsystems
//finite means there's a limited amount of states that a subsystem can be in
//two main finite state machines: one for intake, and one for slides/v4bar/turret