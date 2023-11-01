package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Robot
{
    public Servo claw;
    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        telemetry.addData("Status", "Initialized");
        claw = hardwareMap.get(Servo.class, "clawServo");
        claw.setPosition(0);
        telemetry.update();
    }
    public void log(Telemetry telemetry) {
        telemetry.addData("Claw position", claw.getPosition());
    }
}
@TeleOp

public class servotest extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    boolean pressY,pressA;
    float clawPos=0;
    Robot robot = new Robot();
    public void init() {
        robot.init(telemetry, hardwareMap);
        robot.claw.setPosition(0);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (this.gamepad1.y && pressY == true) {
            pressY = false;
            clawPos+=0.1;
            robot.claw.setPosition(clawPos);
        }
        if (!this.gamepad1.y) pressY = true;

        if (this.gamepad1.a && pressA == true) {
            pressA = false;
            clawPos-=0.1;
            robot.claw.setPosition(clawPos);
        }
        if (!this.gamepad1.a) pressA = true;
    }

    @Override
    public void stop() {
    }
}