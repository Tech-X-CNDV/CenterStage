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

public class CoacazaBlue extends LinearOpMode {

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
        //robot.init(telemetry,hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35.5, 65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        //
        robot.rightSliderMotor = hardwareMap.get(DcMotor.class, "rightSliderMotor");
        robot.leftSliderMotor = hardwareMap.get(DcMotor.class, "leftSliderMotor");

        robot.touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        robot.clawServo = hardwareMap.get(Servo.class, "clawServo");

        robot.rampServo = hardwareMap.get(CRServo.class, "rampServo");
        robot.rampMotor = hardwareMap.get(DcMotor.class, "rampMotor");

        robot.leftServo = hardwareMap.get(Servo.class, "leftServo");
        robot.rightServo = hardwareMap.get(Servo.class, "rightServo");

        robot.leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        robot.rightGrab = hardwareMap.get(Servo.class, "rightGrab");

        robot.leftSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //mid
        /*
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

         */
        Trajectory leftpart1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-29.5, 40), Math.toRadians(-45))
                .build();
        Trajectory leftpart2 = drive.trajectoryBuilder(leftpart1.end(),true)
                .splineTo(new Vector2d(-35.5, 60), Math.toRadians(90))
                .build();
        Trajectory leftpart3 = drive.trajectoryBuilder(leftpart2.end())
                .forward(48.5)
                .build();
        Trajectory leftstack1 = drive.trajectoryBuilder(leftpart3.end(), true)
                .lineToSplineHeading(new Pose2d(-56, 11.5, Math.toRadians(0)))
                .build();

        TrajectorySequence midpart1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-35.5,35))
                .back(10)
                .strafeRight(15)
                .forward(28.5)
                .build();
        Trajectory midstack1 = drive.trajectoryBuilder(midpart1.end(), true)
                .splineTo(new Vector2d(-51, 11.5),Math.toRadians(180))
                .build();

        Trajectory rightpart1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-41.5, 40), Math.toRadians(-135))
                .build();
        Trajectory rightpart2 = drive.trajectoryBuilder(rightpart1.end(),true)
                .splineTo(new Vector2d(-35.5, 60), Math.toRadians(90))
                .build();
        Trajectory rightpart3 = drive.trajectoryBuilder(rightpart2.end())
                .forward(48.5)
                .build();
        Trajectory rightstack1 = drive.trajectoryBuilder(rightpart3.end(), true)
                .lineToSplineHeading(new Pose2d(-56, 11.5, Math.toRadians(0)))
                .build();


        TrajectorySequence stack = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-59, 11.5), Math.toRadians(0)))
                .strafeRight(1)
                .forward(90)
                .build();
        Trajectory midpanel1 = drive.trajectoryBuilder(stack.end())
                .lineTo(new Vector2d(51.5, 35))
                .build();
        Trajectory rightpanel1 = drive.trajectoryBuilder(stack.end())
                .lineTo(new Vector2d(51.5, 30))
                .build();
        Trajectory leftpanel1 = drive.trajectoryBuilder(stack.end())
                .lineTo(new Vector2d(51.5, 45))
                .build();

        Trajectory park = drive.trajectoryBuilder(new Pose2d(new Vector2d(52,15),0))
                        .forward(2).build();
        initTfod();
        // robot.init(telemetry, hardwareMap);
        robot.clawServo.setPosition(0);
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
                drive.followTrajectory(leftpart1);
                drive.followTrajectory(leftpart2);
                drive.followTrajectory(leftpart3);
                drive.followTrajectory(leftstack1);
                break;
            case 2:
                drive.followTrajectory(rightpart1);
                drive.followTrajectory(rightpart2);
                drive.followTrajectory(rightpart3);
                drive.followTrajectory(rightstack1);
                break;
            case 1:
                drive.followTrajectorySequence(midpart1);
                drive.followTrajectory(midstack1);
                break;
        }
        /*
        robot.clawServo.setPosition(0.6);
        robot.intakeToggle();
        robot.Grab();
        sleep(500);
        robot.Grab();
        robot.clawServo.setPosition(0);
        robot.intakeToggle();
        robot.outakeToggle();
        sleep(2500);
        robot.outakeToggle();
                 */
        sleep(5000);
        drive.followTrajectorySequence(stack);
        robot.sliderPos = 700;
        robot.sliderS();
        robot.servoS();
        switch (target) {
            case 0:
                drive.followTrajectory(leftpanel1);
                break;
            case 1:
                drive.followTrajectory(midpanel1);
                break;
            case 2:
                drive.followTrajectory(rightpanel1);
                break;
        }
        sleep(500);
        robot.clawServo.setPosition(0.6);
        sleep(500);
        robot.servoS();
        sleep(200);
        robot.sliderPos = 0;
        robot.sliderS();
        drive.followTrajectory(park);
        /*
        drive.followTrajectorySequence(park);
        sleep(1000);
        robot.clawServo.setPosition(0);
        sleep(1000);
        robot.sliderPos = 800;
        robot.sliderS();
        robot.servoS();

         */
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
