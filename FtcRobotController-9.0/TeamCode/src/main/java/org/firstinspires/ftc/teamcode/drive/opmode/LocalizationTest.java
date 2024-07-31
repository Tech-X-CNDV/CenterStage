package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor rampMotor = hardwareMap.get(DcMotor.class, "rampMotor");
        Servo rampServo = hardwareMap.get(Servo.class, "rampServo");
        Servo arm2Servo = hardwareMap.get(Servo.class, "arm2Servo");
        Servo arm1Servo = hardwareMap.get(Servo.class, "arm1Servo");
        Servo hingeServo = hardwareMap.get(Servo.class, "hingeServo");
        Servo tinderServo = hardwareMap.get(Servo.class, "tinderServo");

        DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1Servo.setPosition(1);
        arm2Servo.setPosition(1);
        rampServo.setDirection(Servo.Direction.REVERSE);
        boolean pressB = false, rampToggle = false, pressA = false, rampReverse = false, rampServoSwitch = false, pressY = false, armServoSwitch = false, pressX = false;
        boolean pressRB = false, tinderServoSwitch = false;
        waitForStart();
        rampServo.setPosition(0);
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            if (gamepad1.b && pressB == true) {
                rampToggle = !rampToggle;
                if (rampToggle) rampMotor.setPower(1);
                else rampMotor.setPower(0);
                pressB = false;
            }
            if (!gamepad1.b) pressB = true;

            if (gamepad1.a && pressA == true) {
                rampReverse = !rampReverse;
                if (rampReverse) rampMotor.setPower(-1);
                else rampMotor.setPower(0);
                pressA = false;
            }
            if (!gamepad1.a) pressA = true;

            if (gamepad1.y && pressY == true) {
                rampServoSwitch = !rampServoSwitch;
                if (rampServoSwitch) hingeServo.setPosition(1);
                else hingeServo.setPosition(0);
                pressY = false;
            }
            if (!gamepad1.y) pressY = true;

            if (gamepad1.x && pressX == true) {
                armServoSwitch = !armServoSwitch;
                if (armServoSwitch) {
                    arm1Servo.setPosition(1);
                    arm2Servo.setPosition(1);
                } else {
                    arm1Servo.setPosition(0);
                    arm2Servo.setPosition(0);
                }
                pressX = false;
            }
            if (!gamepad1.x) pressX = true;

            if (gamepad1.y && pressY == true) {
                rampServoSwitch = !rampServoSwitch;
                if (rampServoSwitch) {
                    hingeServo.setPosition(1);
                    wait(500);
                    arm1Servo.setPosition(0);
                    arm2Servo.setPosition(0);
                } else {
                    hingeServo.setPosition(0);
                    arm1Servo.setPosition(1);
                    arm2Servo.setPosition(1);
                }
                pressY = false;
            }
            if (!gamepad1.y) pressY = true;

            if (gamepad1.right_bumper && pressRB == true) {
                tinderServoSwitch = !tinderServoSwitch;
                if (tinderServoSwitch) tinderServo.setPosition(0.75);
                else tinderServo.setPosition(1);
                pressRB = false;
            }
            if (!gamepad1.right_bumper) pressRB = true;


            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("arm1 pos", arm1Servo.getPosition());
            telemetry.addData("arm2 pos", arm2Servo.getPosition());
            telemetry.addData("hinge pos", hingeServo.getPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
