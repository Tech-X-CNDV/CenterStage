package org.firstinspires.ftc.teamcode;

import android.os.DropBoxManager;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.checkerframework.checker.units.qual.min;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;

class CRobo {
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;

    public DcMotor leftSliderMotor; //invers acelor de ceasornic
    public DcMotor rightSliderMotor;
    public DcMotor rampMotor;
    public Servo rampServo;
    public Servo arm2Servo;
    public Servo arm1Servo;
    public Servo hingeServo;
    public Servo tinderServo;

    public TouchSensor touchSensor;
    public DistanceSensor sensorDistance;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        telemetry.addData("Status", "Initialized");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");

        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        rightSliderMotor = hardwareMap.get(DcMotor.class, "rightSliderMotor");
        leftSliderMotor = hardwareMap.get(DcMotor.class, "leftSliderMotor");

        rampMotor = hardwareMap.get(DcMotor.class, "rampMotor");
        rampServo = hardwareMap.get(Servo.class, "rampServo");
        arm2Servo = hardwareMap.get(Servo.class, "arm2Servo");
        arm1Servo = hardwareMap.get(Servo.class, "arm1Servo");
        hingeServo = hardwareMap.get(Servo.class, "hingeServo");
        tinderServo = hardwareMap.get(Servo.class, "tinderServo");

        DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        arm1Servo.setPosition(0.14);
        arm2Servo.setPosition(0.14);
        //arm1Servo.setDirection(Servo.Direction.REVERSE);
        //arm2Servo.setDirection(Servo.Direction.REVERSE);
        rampServo.setDirection(Servo.Direction.REVERSE);
        rampServo.setPosition(0.1);
        tinderServo.setPosition(0.95);

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.update();
    }

    public int sliderPos, uL = 4000, dL = 0, sliderSpeed = 10;
    public int presetPos = 1700, errorMargin = 25, parkPos = 2200;

    public void sliderS() {
        leftSliderMotor.setTargetPosition(sliderPos);
        rightSliderMotor.setTargetPosition(sliderPos);
    }

    public double armPos;

    public void armsCustom(double coef) {
        armPos += coef;
        arm1Servo.setPosition(armPos);
        arm2Servo.setPosition(armPos);
    }

    public void log(Telemetry telemetry) {

    }
}

@TeleOp

public class nationala extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    double leftStickForward = 0;
    double leftStickSide = 0;
    double botSpin = 0;
    double denominator = 0;
    double frontLeftPower = 0;
    double frontRightPower = 0;
    double rearLeftPower = 0;
    double rearRightPower = 0;
    boolean speedLimit = false;
    boolean pressX = false, pressA = false, pressB = false, pressB1 = false, SCHEMA_LUCA = true, pressA1 = false, pressB2 = false, pressA2 = false,
            pressY = false, pressLbumper2 = false, pressRbumper2 = false, pressLbumper = false, pressRbumper = false, pressDpDown = false, pressDpUp = false, pressDpLeft = false, pressBumperG1 = false;
    boolean rampToggle = false, rampReverse = false, rampServoSwitch = false, armServoSwitch = false;
    boolean pressRB = false, tinderServoSwitch = false, pixelsIn = false, autoLock = false, autoPlace = false;
    double sk = 2000, prevdis, targetdis;
    public boolean tared = false;
    int rampDirection = 1, rampPower = 0;
    double dis;
    CRobo robot = new CRobo();
    static ElapsedTime stopwatch = new ElapsedTime();
    static ElapsedTime retractTimer = new ElapsedTime();

    public void init() {
        //Arrays.fill(dis,0);
        robot.init(telemetry, hardwareMap);
        robot.leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSliderMotor.setTargetPosition(0);
        robot.leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSliderMotor.setTargetPosition(0);
        robot.rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftSliderMotor.setPower(1);
        robot.rightSliderMotor.setPower(1);

    }

    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (gamepad2.left_bumper && pressLbumper2) {
            speedLimit = !speedLimit;
            pressLbumper2 = false;
        }
        if (!gamepad2.left_bumper) pressLbumper2 = true;

        double r = Math.hypot(gamepad2.left_stick_x, -gamepad2.left_stick_y);
        double robotAngle = Math.atan2(-gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad2.right_stick_x;
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        v1 = v1 - v1 * 0.75d * this.gamepad2.right_trigger;
        v2 = v2 - v2 * 0.75d * this.gamepad2.right_trigger;
        v3 = v3 - v3 * 0.75d * this.gamepad2.right_trigger;
        v4 = v4 - v4 * 0.75d * this.gamepad2.right_trigger;

        robot.leftFrontMotor.setPower(v1);
        robot.rightFrontMotor.setPower(v2);
        robot.leftRearMotor.setPower(v3);
        robot.rightRearMotor.setPower(v4);
        //
        // if(gamepad1.left_trigger>0)SCHEMA_LUCA=!SCHEMA_LUCA;

        int curSlider = robot.leftSliderMotor.getCurrentPosition();

        if (gamepad1.left_stick_y != 0 || gamepad1.left_trigger > 0 || gamepad1.a || gamepad1.right_trigger > 0) {
            if (gamepad1.left_stick_y != 0) {
                if (armServoSwitch)
                    robot.sliderPos = Math.max(robot.presetPos + robot.errorMargin, Math.min(robot.uL, (int) (robot.sliderPos - sk * gamepad1.left_stick_y)));
                else
                    robot.sliderPos = Math.max(robot.dL, Math.min(robot.uL, (int) (robot.sliderPos - sk * gamepad1.left_stick_y)));
            }
        } else if (Math.abs(robot.sliderPos - curSlider) > robot.errorMargin)
            robot.sliderPos = curSlider;
        robot.sliderS();
        //robot.armsCustom(gamepad1.right_stick_y/10);
        //
        if (!autoLock) prevdis = dis;
        dis = robot.sensorDistance.getDistance(DistanceUnit.CM);
        if (autoLock) {
            if (prevdis - dis < 0.5d) autoLock = false;
            else {
                if (stopwatch.time() > 0.50d) rampDirection = -1;
                if (stopwatch.time() > 0.75d) {
                    robot.tinderServo.setPosition(0.75);
                    tinderServoSwitch = true;
                    stopwatch.reset();
                    autoLock = false;
                }
            }
        }
        if (prevdis - dis > 0.75d && !autoLock) {
            stopwatch.reset();
            autoLock = true;
        }

        if (curSlider >= robot.presetPos) {
            if (!autoPlace) {
                robot.arm1Servo.setPosition(0.6);
                robot.arm2Servo.setPosition(0.6);
                robot.tinderServo.setPosition(0.75);
                tinderServoSwitch = true;
                armServoSwitch = true;
                autoPlace = true;
            }
        } else autoPlace = false;
        //
        //auto place
            /*
            if (Math.abs(curSlider - robot.presetPos) <= robot.errorMargin) {
                if (!autoPlace) {
                    robot.hingeServo.setPosition(1);
                 //   robot.arm1Servo.setPosition(0);
                 //   robot.arm2Servo.setPosition(0);
                    autoPlace = true;
                }
            } else autoPlace = false;
            /*

             */

        if (this.gamepad1.a && pressA1) {
            robot.arm1Servo.setPosition(0.14);
            robot.arm2Servo.setPosition(0.14);
            armServoSwitch = false;
            retractTimer.reset();
            pressA1 = false;
        }
        if (!this.gamepad1.a) pressA1 = true;
        if (this.gamepad1.a && retractTimer.time() > 1d) {
            robot.sliderPos = 0;
            robot.sliderS();
        }
        if (this.gamepad1.left_trigger > 0) {
            robot.sliderPos = robot.parkPos;
            robot.sliderS();
        }
        if (this.gamepad1.right_trigger > 0) {
            robot.sliderPos = robot.presetPos;
            robot.sliderS();
        }
        if (this.gamepad1.b && pressB1) {
            if (armServoSwitch) {
                robot.arm1Servo.setPosition(0.35);
                robot.arm2Servo.setPosition(0.35);
            } else {
                robot.arm1Servo.setPosition(0.14);
                robot.arm2Servo.setPosition(0.14);
            }
            pressB1 = false;
        }
        if (!this.gamepad1.b) pressB1 = true;

        if (gamepad1.x && pressX == true) {
            armServoSwitch = !armServoSwitch;
            if (armServoSwitch) {
                if (curSlider > robot.presetPos - robot.errorMargin) {
                    robot.arm1Servo.setPosition(0.6);
                    robot.arm2Servo.setPosition(0.6);
                }
            } else {
                robot.arm1Servo.setPosition(0.14);
                robot.arm2Servo.setPosition(0.14);
            }
            pressX = false;
        }
        if (!gamepad1.x) pressX = true;

        if (gamepad2.y && pressY == true) {
            rampServoSwitch = !rampServoSwitch;
            if (rampServoSwitch) {
                robot.rampServo.setPosition(0.6);
                //robot.arm1Servo.setPosition(0);
                //robot.arm2Servo.setPosition(0);
            } else {
                robot.rampServo.setPosition(0.2);
                //robot.arm1Servo.setPosition(1);
                //robot.arm2Servo.setPosition(1);
            }
            pressY = false;
        }
        if (!gamepad2.y) pressY = true;

        robot.rampMotor.setPower(rampPower * rampDirection);
        if (gamepad2.b && pressB2 == true) {
            rampToggle = !rampToggle;
            if (rampToggle){rampPower = 1;rampDirection=1;}
            else {rampPower = 0;rampDirection=1;}
            pressB2 = false;
        }
        if(curSlider>robot.presetPos-robot.errorMargin){rampToggle=false;rampPower=0;rampDirection=1;}
        if (!gamepad2.b) pressB2 = true;

        if (gamepad2.a && pressA2 == true) {
            rampDirection = -1 * rampDirection;
            pressA2 = false;
        }
        if (!gamepad2.a) pressA2 = true;
        /*
        if (gamepad1.y && pressY == true) {
            rampServoSwitch = !rampServoSwitch;
            if (rampServoSwitch) robot.hingeServo.setPosition(1);
            else robot.hingeServo.setPosition(0);
            pressY = false;f
        }
        if (!gamepad1.y) pressY = true;
        */
        /*
        if(gamepad1.dpad_left)
        {
          robot.dL=-10000;
        }
        if(gamepad1.dpad_right)
        {
            robot.dL=0;
            robot.leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftSliderMotor.setTargetPosition(0);
            robot.leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightSliderMotor.setTargetPosition(0);
            robot.rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.sliderPos = 0;
        }

         */
        /*
        if(robot.touchSensor.isPressed() && !tared){
            tared = true;
            robot.dL=0;
            robot.leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftSliderMotor.setTargetPosition(0);
            robot.leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightSliderMotor.setTargetPosition(0);
            robot.rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
         */
        if (gamepad1.right_bumper && pressRB == true) {
            tinderServoSwitch = !tinderServoSwitch;
            if (tinderServoSwitch) robot.tinderServo.setPosition(0.75);
            else {
                robot.tinderServo.setPosition(0.95);
                autoLock = false;
            }
            pressRB = false;
        }
        if (!gamepad1.right_bumper) pressRB = true;
        if (gamepad2.left_trigger > 0) robot.hingeServo.setPosition(1);
        // if (robot.sensorDistance.getDistance(DistanceUnit.CM) < 7.5f) pixelsIn = true;
        // else pixelsIn = false;
        // telemetrie
        robot.log(telemetry);
        telemetry.addData("slider", robot.sliderPos);
        telemetry.addData("Left Stick Y", leftStickForward);
        telemetry.addData("Left Stick X", leftStickSide);
        telemetry.addData("arm1 pos", robot.arm1Servo.getPosition());
        telemetry.addData("arm2 pos", robot.arm2Servo.getPosition());
        telemetry.addData("hinge pos", robot.hingeServo.getPosition());
        telemetry.addData("hinge pos", gamepad1.x);
        //telemetry.addData("range", String.format("%.01f mm", robot.sensorDistance.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", robot.sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("stopwatch", stopwatch.time());
        //telemetry.addData("senzor", pixelsIn);
        // telemetry.addData("target", k);
        telemetry.addData("luca", SCHEMA_LUCA);
        telemetry.addData("touch", robot.touchSensor.isPressed());
        // telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
        //telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));
        robot.log(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}