package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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

public class LucretiaBlue extends LinearOpMode {

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

    CRobot robot = new CRobot();

    @Override

    public void runOpMode() throws InterruptedException {
        robot.init(telemetry, hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, 65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        //mid
        TrajectorySequence midpart1 = drive.trajectorySequenceBuilder(new Pose2d(12, 65, Math.toRadians(-90)))
                .forward(30)
                .back(10)
                .turn(Math.toRadians(90))
                .forward(1)
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 0;
                    robot.sliderS();
                    robot.servoS();
                })
                .splineTo(new Vector2d(52, 36), Math.toRadians(0))
                .build();
        TrajectorySequence midpart2 = drive.trajectorySequenceBuilder(midpart1.end())
                .back(1)
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 0;
                    robot.sliderS();
                    robot.servoS();
                })
                .splineTo(new Vector2d(10, 60), Math.toRadians(180))
                .back(60)
                .strafeLeft(50)
                .build();
        //left
        TrajectorySequence leftpart1 = drive.trajectorySequenceBuilder(new Pose2d(12, 65, Math.toRadians(-90)))
                .splineTo(new Vector2d(18, 40), Math.toRadians(-45))
                .back(7)
                .turn(Math.toRadians(45))
                .forward(1)
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 0;
                    robot.sliderS();
                    robot.servoS();
                })
                .splineTo(new Vector2d(52, 42), Math.toRadians(0))
                .build();
        TrajectorySequence leftpart2 = drive.trajectorySequenceBuilder(leftpart1.end())
                .back(1)
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 0;
                    robot.sliderS();
                    robot.servoS();
                })
                .splineTo(new Vector2d(10, 60), Math.toRadians(180))
                .back(45)
                .strafeLeft(50)
                .build();
        //right
        TrajectorySequence rightpart1 = drive.trajectorySequenceBuilder(new Pose2d(12, 65, Math.toRadians(-90)))
                .splineTo(new Vector2d(6,40 ), Math.toRadians(-135))
                .back(7)
                .turn(Math.toRadians(135))
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 0;
                    robot.sliderS();
                    robot.servoS();
                })
                .splineTo(new Vector2d(52,26 ), Math.toRadians(0))
                .build();
        TrajectorySequence rightpart2 = drive.trajectorySequenceBuilder(rightpart1.end())
                .back(1)
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 0;
                    robot.sliderS();
                    robot.servoS();
                })
                .splineTo(new Vector2d(10,60 ), Math.toRadians(180))
                .back(45)
                .strafeLeft(50)
                .build();
        //final destination
        TrajectorySequence park = drive.trajectorySequenceBuilder(midpart2.end())
                .lineTo(new Vector2d(-60, 11.5))
                .addDisplacementMarker(() -> {
                    robot.intakeToggle();
                    sleep(5000);
                    robot.intakeToggle();
                })
                .forward(70)
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 800;
                    robot.sliderS();
                    robot.servoS();
                })
                .splineTo(new Vector2d(52, 42), Math.toRadians(0))
                .build();
        initTfod();
        robot.init(telemetry, hardwareMap);
        robot.clawServo.setPosition(0.25);
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
                drive.followTrajectorySequence(leftpart1);
                sleep(1000);
                robot.clawServo.setPosition(0);
                sleep(1000);
                drive.followTrajectorySequence(leftpart2);
                break;
            case 2:
                drive.followTrajectorySequence(rightpart1);
                sleep(1000);
                robot.clawServo.setPosition(0);
                sleep(1000);
                drive.followTrajectorySequence(rightpart2);

                break;
            case 1:
                drive.followTrajectorySequence(midpart1);
                sleep(1000);
                robot.clawServo.setPosition(0);
                sleep(1000);
                drive.followTrajectorySequence(midpart2);

                break;
        }
        drive.followTrajectorySequence(park);
        sleep(1000);
        robot.clawServo.setPosition(0);
        sleep(1000);
        robot.sliderPos = 800;
        robot.sliderS();
        robot.servoS();
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

        builder.setAutoStopLiveView(false);
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
