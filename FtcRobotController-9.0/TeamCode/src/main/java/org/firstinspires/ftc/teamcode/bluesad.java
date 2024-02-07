package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

public class bluesad extends LinearOpMode {

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

    public void runOpMode() throws InterruptedException{
        robot.init(telemetry, hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        //
        TrajectorySequence rightSpike = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d( 6,35 ), Math.toRadians(-135))
                .build();
        Trajectory rightBack = drive.trajectoryBuilder(rightSpike.end(), true)
                .splineTo(new Vector2d( 12,55 ), Math.toRadians(90))
                .build();
        TrajectorySequence rightRepo = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(rightBack.end().getX(),rightBack.end().getY()),Math.toRadians(0)))
                .forward(84)
                .build();
        TrajectorySequence rightPark0 = drive.trajectorySequenceBuilder(rightRepo.end())
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 1200;
                    robot.sliderS();
                    robot.servoS();
                })
                .strafeLeft(40)
                .forward(4)
                .build();
        TrajectorySequence rightPark1 = drive.trajectorySequenceBuilder(rightPark0.end())
                .back(5)
                .build();
        TrajectorySequence rightPark2 = drive.trajectorySequenceBuilder(rightPark1.end())
                .strafeLeft(23)
                .build();
        TrajectorySequence rightPark3 = drive.trajectorySequenceBuilder(rightPark2.end())
                .forward(16)
                .build();
        //
        TrajectorySequence midSpike = drive.trajectorySequenceBuilder(startPose)
                .forward(30)
                .build();
        TrajectorySequence midBack = drive.trajectorySequenceBuilder(midSpike.end())
                .back(25)
                .build();
        TrajectorySequence midRepo = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(midBack.end().getX(),midBack.end().getY()),Math.toRadians(0)))
                .forward(84)
                .build();
        TrajectorySequence midPark = drive.trajectorySequenceBuilder(midRepo.end())
                .addDisplacementMarker(() -> {
                    robot.servoS();
                })
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 1200;
                    robot.sliderS();
                })
                .strafeLeft(32)
                .forward(4)
                .build();
        TrajectorySequence midPark0 = drive.trajectorySequenceBuilder(midPark.end())
                .back(5)
                .build();
        TrajectorySequence midPark1 = drive.trajectorySequenceBuilder(midPark0.end())
                .strafeLeft(32)
                .build();
        TrajectorySequence midPark2 = drive.trajectorySequenceBuilder(midPark1.end())
                .forward(15)
                .build();
        //
        TrajectorySequence leftSpike = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d( 18,35 ), Math.toRadians(-45))
                .build();
        Trajectory leftBack = drive.trajectoryBuilder(leftSpike.end(), true)
                .splineTo(new Vector2d( 12,55 ), Math.toRadians(90))
                .build();
        TrajectorySequence leftRepo = drive.trajectorySequenceBuilder( new Pose2d(new Vector2d(leftBack.end().getX(),leftBack.end().getY()),Math.toRadians(0)))
                .forward(84)
                .build();
        TrajectorySequence leftPark0 = drive.trajectorySequenceBuilder(leftRepo.end())
                .addDisplacementMarker(() -> {
                    robot.servoS();
                })
                .strafeLeft(23)
                .addDisplacementMarker(() -> {
                    robot.sliderPos = 1200;
                    robot.sliderS();
                })
                .forward(4)
                .build();
        TrajectorySequence leftPark1 = drive.trajectorySequenceBuilder(leftPark0.end())
                .back(5)
                .build();
        TrajectorySequence leftPark2 = drive.trajectorySequenceBuilder(leftPark1.end())
                .strafeLeft(42)
                .build();
        TrajectorySequence lPark = drive.trajectorySequenceBuilder(leftPark2.end())
                .forward(15)
                .build();
        //final destination
        initTfod();
        robot.init(telemetry, hardwareMap);
        robot.clawServo.setPosition(0.25);
        while (!isStarted() && !isStopRequested()) {

            telemetryTfod();

            telemetry.update();

        }
        waitForStart();
        visionPortal.close();
        robot.leftSliderMotor.setTargetPosition(0);
        robot.leftSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSliderMotor.setTargetPosition(0);
        robot.rightSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSliderMotor.setPower(1);
        robot.rightSliderMotor.setPower(1);
        //roadrunner
        switch (target) {
            case 0:
                // traiectorie pt stanga spike back repo park
                robot.clawServo.setPosition(0);
                drive.followTrajectorySequence(leftSpike);
                drive.followTrajectory(leftBack);
                sleep(5000);
                drive.turn(Math.toRadians(90));
                //
                drive.followTrajectorySequence(leftRepo);
                drive.followTrajectorySequence(leftPark0);
                sleep(3000);
                robot.clawServo.setPosition(0.25);
                sleep(3000);
                robot.sliderPos = 0;
                robot.sliderS();
                robot.servoS();
                drive.followTrajectorySequence(leftPark1);
                robot.sliderPos = 0;
                robot.sliderS();
                drive.followTrajectorySequence(leftPark2);
                drive.followTrajectorySequence(lPark);
                break;
            case 2:
                // traiectorie pt dreapta
                robot.clawServo.setPosition(0);
                drive.followTrajectorySequence(rightSpike);
                drive.followTrajectory(rightBack);
                sleep(5000);
                drive.turn(Math.toRadians(90));
                //
                drive.followTrajectorySequence(rightRepo);
                drive.followTrajectorySequence(rightPark0);
                sleep(3000);
                robot.clawServo.setPosition(0.25);
                sleep(3000);
                robot.sliderPos = 0;
                robot.sliderS();
                robot.servoS();
                drive.followTrajectorySequence(rightPark1);
                drive.followTrajectorySequence(rightPark2);
                drive.followTrajectorySequence(rightPark3);
                break;
            case 1:
                // traiectorie pt mijloc
                robot.clawServo.setPosition(0);
                drive.followTrajectorySequence(midSpike);
                drive.followTrajectorySequence(midBack);
                sleep(5000);
                drive.turn(Math.toRadians(90));
                //
                drive.followTrajectorySequence(midRepo);
                drive.followTrajectorySequence(midPark);
                sleep(3000);
                robot.clawServo.setPosition(0.25);
                sleep(3000);
                robot.sliderPos = 0;
                robot.sliderS();
                robot.servoS();
                drive.followTrajectorySequence(midPark1);
                drive.followTrajectorySequence(midPark2);
                break;
        }
    }   // end runOpMode(
    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
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
        tfod.setMinResultConfidence(0.75f);
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
