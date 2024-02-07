package org.firstinspires.ftc.teamcode;

import android.os.DropBoxManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;

class CRobot {
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;

    public DcMotor leftSliderMotor; //invers acelor de ceasornic
    public DcMotor rightSliderMotor;
    public Servo clawServo;

    public Servo leftServo;
    public Servo rightServo;

    public CRServo rampServo;
    public DcMotor rampMotor;

    public Servo planeServo;
    //public DcMotor planeMotor;

    public TouchSensor touchSensor;

    public Servo leftGrab;
    public Servo rightGrab;
    //public Servo chainServo;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        telemetry.addData("Status", "Initialized");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");

        rightSliderMotor = hardwareMap.get(DcMotor.class, "rightSliderMotor");
        leftSliderMotor = hardwareMap.get(DcMotor.class, "leftSliderMotor");

        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        clawServo = hardwareMap.get(Servo.class, "clawServo");

        rampServo = hardwareMap.get(CRServo.class, "rampServo");
        rampMotor = hardwareMap.get(DcMotor.class, "rampMotor");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");

        planeServo = hardwareMap.get(Servo.class, "planeServo");
        // chainServo = hardwareMap.get(Servo.class, "chainServo");


        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rampMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //planeMotor = hardwareMap.get(DcMotor.class, "planeMotor");

        //leftServo.setDirection(Servo.Direction.REVERSE);
        // rightServo.setDirection(Servo.Direction.REVERSE);
        telemetry.update();
    }

    public void plane() {
        planeServo.setPosition(0);
    }

    public boolean grabPow;

    public void Grab() {
        grabPow = !grabPow;
        if (grabPow) {
            leftGrab.setPosition(1);
            rightGrab.setPosition(0);
        } else {
            leftGrab.setPosition(0);
            rightGrab.setPosition(1);
        }

    }

    public void noEncoderSlider(float pos) {
        pos = pos * -1;
        if (!touchSensor.isPressed() && pos < 0)
            leftSliderMotor.setPower(pos);
        rightSliderMotor.setPower(pos);
        if (pos >= 0) leftSliderMotor.setPower(pos);
        rightSliderMotor.setPower(pos);
    }

    public boolean Pow;

    public void intakeToggle() {
        Pow = !Pow;
        if (Pow) {
            rampMotor.setPower(1);
            rampServo.setPower(-1);
        } else {
            rampMotor.setPower(0);
            rampServo.setPower(0);
        }
    }

    public boolean krPow;

    public void outakeToggle() {
        krPow = !krPow;
        if (krPow) {
            rampMotor.setPower(-1);
            rampServo.setPower(1);
        } else {
            rampMotor.setPower(0);
            rampServo.setPower(0);
        }
    }

    public void limitUp() {
        uL += 100;
    }

    public void limitDown() {
        dL -= 100;
    }

    public int sliderPos, uL = 4000, dL = 0, sliderSpeed = 10;

    public void sliderS() {
        leftSliderMotor.setTargetPosition(sliderPos);
        rightSliderMotor.setTargetPosition(sliderPos);
    }

    public boolean clawPow = false;

    public void clawServo() {
        clawPow = !clawPow;
        if (clawPow) {
            clawServo.setPosition(0.25);
        } else {
            clawServo.setPosition(0);
        }
    }

    public void clawServoCustom(float pos) {

        clawServo.setPosition(Math.min(pos, 0.5));
    }

    public boolean servoPow;

    public void servoS() {
        servoPow = !servoPow;
        if (servoPow) {
            leftServo.setPosition(1);
            rightServo.setPosition(0);
        } else {
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        }
    }

    public void planeLaunch(float p) {
        //  planeMotor.setPower(-p);
    }

    public void antiHeat() {
        if (sliderPos == 0 && leftSliderMotor.getCurrentPosition() < 25) {
            rightSliderMotor.setPower(0);
            leftSliderMotor.setPower(0);
            leftSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            rightSliderMotor.setPower(1);
            leftSliderMotor.setPower(1);
            leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (touchSensor.isPressed() && sliderPos == 0) {
            leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void chainS(float pos) {
        // chainServo.setPosition(Math.min(pos, 0.3));
    }

    public void log(Telemetry telemetry) {

    }
}

@TeleOp

public class test extends OpMode {
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
    boolean pressX = false, pressA = false, pressB = false, pressB1 = false,
            pressY = false, pressLbumper2 = false, pressRbumper2 = false, pressLbumper = false, pressRbumper = false, pressDpDown = false, pressDpUp = false, pressDpLeft = false, pressBumperG1 = false;
    double sk = 20;
    CRobot robot = new CRobot();

    public void init() {
        robot.init(telemetry, hardwareMap);

        robot.leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSliderMotor.setTargetPosition(0);
        robot.leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSliderMotor.setTargetPosition(0);
        robot.rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftSliderMotor.setPower(1);
        robot.rightSliderMotor.setPower(1);

        robot.leftServo.setPosition(0);
        robot.rightServo.setPosition(1);
        robot.planeServo.setPosition(1);
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

        leftStickForward = this.gamepad2.left_stick_y;
        leftStickSide = -this.gamepad2.left_stick_x;
        botSpin = -this.gamepad2.right_stick_x;
        //speed toggle
        if (speedLimit) {
            leftStickForward = .25 * this.gamepad2.left_stick_y;
            leftStickSide = -.25 * this.gamepad2.left_stick_x;
            botSpin = -.25 * this.gamepad2.right_stick_x;
        }
        if (gamepad2.left_bumper && pressLbumper2) {
            speedLimit = !speedLimit;
            pressLbumper2 = false;
        }
        if (!gamepad2.left_bumper) pressLbumper2 = true;
        //movement
        denominator = Math.max(Math.abs(leftStickForward) + Math.abs(leftStickSide) + Math.abs(botSpin), 1);
        frontLeftPower = (leftStickForward + leftStickSide + botSpin) / denominator;
        rearRightPower = (leftStickForward + leftStickSide - botSpin) / denominator;
        frontRightPower = (leftStickForward - leftStickSide - botSpin) / denominator;
        rearLeftPower = (leftStickForward - leftStickSide + botSpin) / denominator;

       // robot.leftFrontMotor.setPower(frontLeftPower);
        //robot.rightRearMotor.setPower(rearRightPower);

        //robot.rightFrontMotor.setPower(frontRightPower);
        //robot.leftRearMotor.setPower(rearLeftPower);
        double r = Math.hypot(gamepad2.left_stick_x, gamepad2.left_stick_y);
        double robotAngle = Math.atan2(gamepad2.left_stick_y, gamepad2 .left_stick_x) - Math.PI / 4;
        double rightX = gamepad2.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.leftFrontMotor.setPower(v1);
        robot.rightFrontMotor.setPower(v2);
        robot.leftRearMotor.setPower(v3);
        robot.rightRearMotor.setPower(v4);
        // control
        if (this.gamepad1.right_bumper && pressRbumper) {
            robot.Grab();
            pressRbumper = false;
        }
        if (!this.gamepad1.right_bumper) pressRbumper = true;

        //if (this.gamepad1.b && pressB1) {
        //  robot.clawServo();
        // pressB1 = false;
        //}
        //if (!this.gamepad1.b) pressB1 = true;
        robot.clawServoCustom(1-gamepad1.right_trigger);
        if (this.gamepad2.right_bumper && pressRbumper2) {
            robot.intakeToggle();
            pressRbumper2 = false;
        }
        if (!this.gamepad2.right_bumper) pressRbumper2 = true;

        if (this.gamepad2.b && pressB) {
            robot.outakeToggle();
            pressB = false;
        }
        if (!this.gamepad2.b) pressB = true;
        //
        robot.planeLaunch(this.gamepad1.left_trigger);

        if (gamepad1.left_stick_y != 0)
            robot.sliderPos = Math.max(robot.dL, Math.min(robot.uL, (int) (robot.sliderPos - sk * gamepad1.left_stick_y)));
        else robot.sliderPos = robot.leftSliderMotor.getCurrentPosition();
        robot.sliderS();
        // robot.antiHeat();
        //robot.noEncoderSlider(gamepad1.left_stick_y);

        if (this.gamepad1.a && pressA) {
            robot.sliderPos = 0;
            robot.sliderS();
            pressA = false;
        }
        if (!this.gamepad1.a) pressA = true;

        if (this.gamepad1.dpad_up && pressDpUp) {
            robot.limitUp();
            pressDpUp = false;
        }
        if (!this.gamepad1.dpad_up) pressDpUp = true;

        if (this.gamepad1.dpad_down && pressDpDown) {
            robot.limitDown();
            pressDpDown = false;
        }
        if (!this.gamepad1.dpad_down) pressDpDown = true;


        if (!this.gamepad1.dpad_left) pressDpLeft = true;

        if (this.gamepad1.x && pressX == true) {
            robot.servoS();
            pressX = false;
        }
        if (!this.gamepad1.x) pressX = true;

        if (gamepad1.left_trigger > 0 && (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0))
            robot.plane();
        if (gamepad2.y && pressY == true) {
            robot.Grab();
            pressY = false;
        }
        if(!gamepad2.y)pressY=true;
        // telemetrie
        robot.log(telemetry);
        telemetry.addData("Left Stick Y", leftStickForward);
        telemetry.addData("Left Stick X", leftStickSide);
        telemetry.addData("clawServo pos", robot.clawServo.getPosition());
        //telemetry.addData("hinge pos", robot.hingeServo.getPosition());
        telemetry.addData("slider target", robot.leftSliderMotor.getTargetPosition());
        telemetry.addData("left", 1 - robot.leftServo.getPosition());
        telemetry.addData("right", robot.rightServo.getPosition());
        //.addData("hinge", robot.hingeServo.getPosition());
        telemetry.addData("ramp", robot.rampMotor.getPower());
        telemetry.addData("heat", robot.leftSliderMotor.getPower());
        robot.log(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}