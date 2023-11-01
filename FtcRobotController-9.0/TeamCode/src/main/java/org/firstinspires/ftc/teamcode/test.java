package org.firstinspires.ftc.teamcode;

import android.os.DropBoxManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    public Servo chainServo;
    public DcMotor chainMotor;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        telemetry.addData("Status", "Initialized");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");

        rightSliderMotor = hardwareMap.get(DcMotor.class, "rightSliderMotor");
        leftSliderMotor = hardwareMap.get(DcMotor.class, "leftSliderMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        chainMotor = hardwareMap.get(DcMotor.class, "chainMotor");
        chainServo = hardwareMap.get(Servo.class, "chainServo");

        leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.update();
    }

    public boolean chPow;

    public void chainPower() {
        chPow = !chPow;
        if (chPow) chainMotor.setPower(1);
        else chainMotor.setPower(0);
    }

    public double chainPos = 0, chK = 0.1;
    public void chainS(){chainServo.setPosition(chainPos);}

    public void chainUp() {
        if (chainPos < 1) chainPos += chK;
        chainS();
    }

    public void chainDown() {
        if (chainPos > 0) chainPos -= chK;
        chainS();
    }

    public int sliderPos,uL,dL,sliderSpeed;

    public void sliderS(){
        leftSliderMotor.setTargetPosition(sliderPos);
        rightSliderMotor.setTargetPosition(sliderPos);
    }
    public void sliderUp() {
        if(sliderPos<uL)sliderPos+=sliderSpeed;
        sliderS();
    }

    public void sliderDown() {
        if(sliderPos>dL)sliderPos-=sliderSpeed;
        sliderS();
    }

    public boolean clawPow;
    public void claw() {
        clawPow = !clawPow;
        if (clawPow) clawServo.setPosition(1);
        else chainMotor.setPower(0);
    }

    public void log(Telemetry telemetry) {

    }
}

@TeleOp

public class test extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public double leftStickForward = 0;
    public double leftStickSide = 0;
    public double botSpin = 0;
    public double denominator = 0;
    public double frontLeftPower = 0;
    public double frontRightPower = 0;
    public double rearLeftPower = 0;
    public double rearRightPower = 0;
    public boolean speedLimit=false;
    boolean pressX = false, pressA = false,pressB = false, pressY = false, pressLbumper = false, pressRbumper = false, pressDpDown = false, pressDpUp = false, pressDpLeft = false, pressBumperG1 = false;
    CRobot robot = new CRobot();
    public double gear;

    public void init() {
        robot.init(telemetry, hardwareMap);
        robot.leftSliderMotor.setTargetPosition(0);
        robot.leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightSliderMotor.setTargetPosition(0);
        robot.rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.clawServo.setPosition(0);
        robot.chainServo.setPosition(0);


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

        leftStickForward = -this.gamepad2.left_stick_y;
        leftStickSide = this.gamepad2.left_stick_x;
        botSpin = this.gamepad2.right_stick_x;
        if (speedLimit) {
            leftStickForward = .25 * -this.gamepad2.left_stick_y;
            leftStickSide = .25 * this.gamepad2.left_stick_x;
            botSpin = .25 * this.gamepad2.right_stick_x;
        }
        //movement
        denominator = Math.max(Math.abs(leftStickForward) + Math.abs(leftStickSide) + Math.abs(botSpin), 1);
        frontLeftPower = (leftStickForward + leftStickSide + botSpin) / denominator;
        rearRightPower = (leftStickForward + leftStickSide - botSpin) / denominator;
        frontRightPower = (leftStickForward - leftStickSide - botSpin) / denominator;
        rearLeftPower = (leftStickForward - leftStickSide + botSpin) / denominator;

        robot.leftFrontMotor.setPower(frontLeftPower);
        robot.rightRearMotor.setPower(rearRightPower);

        robot.rightFrontMotor.setPower(frontRightPower);
        robot.leftRearMotor.setPower(rearLeftPower);
        // control

        //lift override (teleop)
        //chain position
        if (this.gamepad1.y && pressY == true) {
            pressY = false;
            robot.chainUp();
        }
        if (!this.gamepad1.y) pressY = true;

        if (this.gamepad1.a && pressA == true) {
            pressA = false;
            robot.chainDown();
        }
        if (!this.gamepad1.a) pressA = true;

        if (this.gamepad1.b && pressB) {
            robot.chainPower();
            pressDpDown = false;
        }
        if (!this.gamepad1.b) pressB = true;

        if (this.gamepad2.right_trigger > 0) {robot.sliderUp();}
        if (this.gamepad2.left_trigger > 0 ) {robot.sliderDown();}

        if (this.gamepad1.x && pressX == true) {
            robot.claw();
            pressX = false;
        }
        if (!this.gamepad1.x) pressX = true;
        //if (this.gamepad1.back) robot.resetSlider();

        // telemetrie
        robot.log(telemetry);
        telemetry.addData("Left Stick Y", leftStickForward);
        telemetry.addData("Left Stick X", leftStickSide);
        robot.log(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}