package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous

public class backdropBlue extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/blue.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private int camerax = 1280;
    private int cameray = 720;
    private int target = 0;

    CRobo robot = new CRobo();

    @Override

    public void runOpMode() throws InterruptedException {
        //robot.init(telemetry,hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(11.5, 65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        //
        robot.rightSliderMotor = hardwareMap.get(DcMotor.class, "rightSliderMotor");
        robot.leftSliderMotor = hardwareMap.get(DcMotor.class, "leftSliderMotor");

        robot.touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        robot.tinderServo = hardwareMap.get(Servo.class, "tinderServo");
        //robot.clawServo = hardwareMap.get(Servo.class, "clawServo");

        robot.rampServo = hardwareMap.get(Servo.class, "rampServo");
        robot.rampMotor = hardwareMap.get(DcMotor.class, "rampMotor");

        robot.arm1Servo = hardwareMap.get(Servo.class, "arm1Servo");
        robot.arm2Servo = hardwareMap.get(Servo.class, "arm2Servo");

        //robot.leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        //robot.rightGrab = hardwareMap.get(Servo.class, "rightGrab");
        //mid
        robot.leftSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(10)
                .splineTo(new Vector2d(3, 40), Math.toRadians(-135))
                .setReversed(true)
                .splineTo(new Vector2d(11.5, 50), Math.toRadians(90))
                .waitSeconds(5)
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    robot.leftSliderMotor.setTargetPosition(robot.presetPos + 50);
                    robot.rightSliderMotor.setTargetPosition(robot.presetPos + 50);
                })
                .splineTo(new Vector2d(44, 34.5), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.arm1Servo.setPosition(0.5);
                    robot.arm2Servo.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .forward(0.1)
                .addDisplacementMarker(() -> {
                    robot.tinderServo.setPosition(0.95);
                })
                .setReversed(true)
                .waitSeconds(1)
                .forward(0.1)
                .addDisplacementMarker(() -> {
                    robot.arm1Servo.setPosition(0.14);
                    robot.arm2Servo.setPosition(0.14);
                })
                .waitSeconds(1)
                .UNSTABLE_addDisplacementMarkerOffset(15, () -> {
                    robot.leftSliderMotor.setTargetPosition(0);
                    robot.rightSliderMotor.setTargetPosition(0);
                })
                .lineTo(new Vector2d(43, 65))
                .build();
        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(10)
                .splineTo(new Vector2d(19, 40), Math.toRadians(-45))
                .setReversed(true)
                .splineTo(new Vector2d(11.5, 60), Math.toRadians(90))
                .waitSeconds(5)
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    robot.leftSliderMotor.setTargetPosition(robot.presetPos + 50);
                    robot.rightSliderMotor.setTargetPosition(robot.presetPos + 50);
                })
                .splineTo(new Vector2d(44, 49), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.arm1Servo.setPosition(0.5);
                    robot.arm2Servo.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .forward(0.1)
                .addDisplacementMarker(() -> {
                    robot.tinderServo.setPosition(0.95);
                })
                .setReversed(true)
                .waitSeconds(1)
                .forward(0.1)
                .addDisplacementMarker(() -> {
                    robot.arm1Servo.setPosition(0.14);
                    robot.arm2Servo.setPosition(0.14);
                })
                .waitSeconds(1)
                .UNSTABLE_addDisplacementMarkerOffset(15, () -> {
                    robot.leftSliderMotor.setTargetPosition(0);
                    robot.rightSliderMotor.setTargetPosition(0);
                })
                .lineTo(new Vector2d(43, 65))
                .build();
        TrajectorySequence purpleMid = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(30)
                .back(20)
                .waitSeconds(5)
                .addDisplacementMarker(() -> {
                    robot.leftSliderMotor.setTargetPosition(robot.presetPos + 50);
                    robot.rightSliderMotor.setTargetPosition(robot.presetPos + 50);
                })
                .setReversed(false)
                .splineTo(new Vector2d(44, 41.5), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.arm1Servo.setPosition(0.5);
                    robot.arm2Servo.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .forward(0.1)
                .addDisplacementMarker(() -> {
                    robot.tinderServo.setPosition(0.95);
                })
                .setReversed(true)
                .waitSeconds(1)
                .forward(0.1)
                .addDisplacementMarker(() -> {
                    robot.arm1Servo.setPosition(0.14);
                    robot.arm2Servo.setPosition(0.14);
                })
                .waitSeconds(1)
                .UNSTABLE_addDisplacementMarkerOffset(15, () -> {
                    robot.leftSliderMotor.setTargetPosition(0);
                    robot.rightSliderMotor.setTargetPosition(0);
                })
                .lineTo(new Vector2d(43, 65))
                .build();
        TrajectorySequence whitePixel = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(25, 15), 0))
                .back(95)
                .addDisplacementMarker(90, () -> {
                    robot.rampServo.setPosition(1);
                })
                .addDisplacementMarker(() -> {
                    robot.rampMotor.setPower(1);
                })
                .strafeRight(38)
                .waitSeconds(2)
                .forward(110)
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
                    robot.rampMotor.setPower(-1);
                })
                .UNSTABLE_addDisplacementMarkerOffset(30, () -> {
                    robot.tinderServo.setPosition(0.75);
                    robot.rampMotor.setPower(0);
                })
                .build();
                /*
                .strafeRight(25)
                .UNSTABLE_addDisplacementMarkerOffset(5,()->{
                    robot.rampMotor.setPower(1);
                })
                .back(10)
                 */

        initTfod();
        // robot.init(telemetry, hardwareMap);
        // robot.clawServo.setPosition(0);
        robot.arm1Servo.setPosition(0.14);
        robot.arm2Servo.setPosition(0.14);
        robot.rampServo.setDirection(Servo.Direction.REVERSE);
        robot.rampServo.setPosition(0.1);
        robot.tinderServo.setPosition(0.75);
        while (!isStarted() && !isStopRequested()) {

            telemetryTfod();

            telemetry.update();

        }
        waitForStart();
        visionPortal.close();

        robot.leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSliderMotor.setTargetPosition(0);
        robot.leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSliderMotor.setTargetPosition(0);
        robot.rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSliderMotor.setPower(1);
        robot.rightSliderMotor.setPower(1);

        //roadrunner
        switch (target) {
            case 0:
                drive.followTrajectorySequence(purpleLeft);
                break;
            case 1:
                drive.followTrajectorySequence(purpleMid);
                break;
            case 2:
                drive.followTrajectorySequence(purpleRight);
                break;
        }
        while (!isStopRequested()){
            robot.leftSliderMotor.setTargetPosition(0);
        robot.rightSliderMotor.setTargetPosition(0);
    }
        //  drive.followTrajectorySequence(whitePixel);
    }   // end runOpMode(

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by usin a builder.
        tfod = new TfodProcessor.Builder()
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(camerax, cameray));

        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        builder.setAutoStopLiveView(true);
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.6f);
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        if (currentRecognitions.size() != 0) {
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                if (recognition.getLabel() == "Pixel") {
                    if (x < camerax / 2) target = 1;
                    else target = 2;
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    break;
                }
            }
        } else target = 0;
        //else target = "left";
        telemetry.addData("Target", target);
    }

}
