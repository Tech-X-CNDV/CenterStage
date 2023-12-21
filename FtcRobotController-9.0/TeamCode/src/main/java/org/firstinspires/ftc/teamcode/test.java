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
    //public Servo clawServo;
    //public Servo hingeServo;
    public CRServo tinderServo;

    public Servo leftServo;
    public Servo rightServo;

    //public Servo chainServo;
    public DcMotor chainMotor;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        telemetry.addData("Status", "Initialized");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");

        rightSliderMotor = hardwareMap.get(DcMotor.class, "rightSliderMotor");
        leftSliderMotor = hardwareMap.get(DcMotor.class, "leftSliderMotor");
        //clawServo = hardwareMap.get(Servo.class, "clawServo");
        //hingeServo = hardwareMap.get(Servo.class, "hingeServo");
        tinderServo = hardwareMap.get(CRServo.class, "tinderServo");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        chainMotor = hardwareMap.get(DcMotor.class, "chainMotor");

        // chainServo = hardwareMap.get(Servo.class, "chainServo");

        leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);

       // leftServo.setDirection(Servo.Direction.REVERSE);
        //rightServo.setDirection(Servo.Direction.REVERSE);
        telemetry.update();
    }

    public boolean chPow;

    public void chainPower() {
        chPow = !chPow;
        if (chPow) chainMotor.setPower(1);
        else chainMotor.setPower(0);
    }

    /*
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
    */
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

    public void sliderUp() {
        if (sliderPos < uL) sliderPos += sliderSpeed;
        sliderS();
    }

    public void sliderDown() {
        if (sliderPos > dL) sliderPos -= sliderSpeed;
        sliderS();
    }

    public boolean clawPow;

    /*
    public void claw() {
        clawPow = !clawPow;
        if (clawPow) clawServo.setPosition(0.5);
        else clawServo.setPosition(0.57);
    }
    */
    public boolean tinderPow;

   // public void tinder() {
      //  tinderPow = !tinderPow;
      //  if (tinderPow) tinderServo.setPosition(1);
       // else tinderServo.setPosition(0);
   // }
   public void tinder() {
       tinderPow = !tinderPow;
       if (tinderPow) tinderServo.setPower(0);
       else tinderServo.setPower(1);
   }
    /*
    public void clawCustom(float pos) {
        clawServo.setPosition(pos);
    }

    public void hingeCustom(float pos) {
        hingeServo.setPosition(pos);
    }
    */
    public void tinderCustom(float pos) {
        tinderServo.setPower(pos);
    }

    public void servoCustom(float pos) {
        rightServo.setPosition(1-pos);
        leftServo.setPosition(pos);
    }

    public boolean servoPow;
    public void servoS() {
        servoPow = !servoPow;
        if (servoPow){ leftServo.setPosition(1);rightServo.setPosition(0);}
        else {leftServo.setPosition(0);rightServo.setPosition(1);}
    }
    public boolean hingePow;
    /*
    public void hinge() {
        hingePow = !hingePow;
        if (hingePow) hingeServo.setPosition(1);
        else hingeServo.setPosition(0);
    }

    public void failSafe()
    {
        if(sliderPos <1200) {
            clawPow = false;
            clawServo.setPosition(0.57);
        }
    }
    */
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
    boolean pressX = false, pressA = false, pressB = false, pressY = false, pressLbumper2 = false, pressLbumper = false, pressRbumper = false, pressDpDown = false, pressDpUp = false, pressDpLeft = false, pressBumperG1 = false;
    double sk=20;
    CRobot robot = new CRobot();

    public void init() {
        robot.init(telemetry, hardwareMap);
        robot.leftSliderMotor.setTargetPosition(0);
        robot.leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightSliderMotor.setTargetPosition(0);
        robot.rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSliderMotor.setPower(1);
        robot.rightSliderMotor.setPower(1);

        robot.leftServo.setPosition(0);
        robot.rightServo.setPosition(1);
        //robot.clawServo.setPosition(0.57);
        // robot.chainServo.setPosition(0);
        //robot.hingeServo.setPosition(0);
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
        botSpin = -this.gamepad2.right_stick_x;
        //speed toggle
        if (speedLimit) {
            leftStickForward = .25 * -this.gamepad2.left_stick_y;
            leftStickSide = .25 * this.gamepad2.left_stick_x;
            botSpin = .25 * this.gamepad2.right_stick_x;
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

        robot.leftFrontMotor.setPower(frontLeftPower);
        robot.rightRearMotor.setPower(rearRightPower);

        robot.rightFrontMotor.setPower(frontRightPower);
        robot.leftRearMotor.setPower(rearLeftPower);
        // control
        //chain position
        /*
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
        */
        //robot.clawCustom(this.gamepad1.left_stick_y);
        // robot.hingeCustom(this.gamepad1.right_stick_y);
        //robot.tinderCustom(this.gamepad1.right_stick_y);

        //robot.servoCustom(this.gamepad1.right_stick_x);

        if (this.gamepad1.y && pressY) {
            robot.chainPower();
            pressY = false;
        }
        if (!this.gamepad1.y) pressY = true;

        //if (this.gamepad1.right_trigger > 0) {robot.sliderUp();}
        //if (this.gamepad1.left_trigger > 0 ) {robot.sliderDown();}
        robot.sliderPos = Math.max(robot.dL, Math.min(robot.uL, (int)(robot.sliderPos - sk * gamepad1.left_stick_y)));
        robot.sliderS();
        if(this.gamepad1.a && pressA)
        {
            robot.sliderPos = 0;
            robot.sliderS();
            pressA=false;
        }
        if(!this.gamepad1.a)pressA=true;

        if (this.gamepad1.dpad_up && pressDpUp) {
            robot.limitUp();
            pressDpUp = false;
        }
        if (!this.gamepad1.dpad_up) pressDpUp = true;

        if (this.gamepad1.dpad_down && pressDpDown) {
            robot.limitUp();
            pressDpDown = false;
        }
        if (!this.gamepad1.dpad_down) pressDpDown = true;

        if (this.gamepad1.x && pressX == true) {
            robot.servoS();
            pressX = false;
        }
        if (!this.gamepad1.x) pressX = true;

        if (this.gamepad1.b && pressB == true) {
            robot.tinder();
            pressB = false;
        }
        if (!this.gamepad1.b) pressB = true;

        //robot.failSafe();
        //if (this.gamepad1.back) robot.resetSlider();

        // telemetrie
        robot.log(telemetry);
        telemetry.addData("Left Stick Y", leftStickForward);
        telemetry.addData("Left Stick X", leftStickSide);
        // telemetry.addData("claw pos", robot.clawServo.getPosition());
        //telemetry.addData("hinge pos", robot.hingeServo.getPosition());
        telemetry.addData("slider target", robot.leftSliderMotor.getTargetPosition());
        telemetry.addData("left", 1-robot.leftServo.getPosition());
        telemetry.addData("right", robot.rightServo.getPosition());
        //.addData("hinge", robot.hingeServo.getPosition());
        telemetry.addData("chain", robot.chainMotor.getPower());
        robot.log(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}