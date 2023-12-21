//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
////import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.apriltag.AprilTagDetection;
//
//import java.util.ArrayList;
//
//@Autonomous
//
//public class autonomie_test extends LinearOpMode {
//    OpenCvCamera camera;
//    WebcamName webcamName;
//    static final double FEET_PER_METER = 3.28084;
//    double fx = 925.2352;
//    double fy = 925.2352;
//    double cx = 643.432;
//    double cy = 355.991785711;
//    /*
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//    */
//    double tagsize = 0.035;
//
//    int ID_TAG_OF_INTEREST_0 = 0;
//    int ID_TAG_OF_INTEREST_1 = 9;
//    int ID_TAG_OF_INTEREST_2 = 19;
//    AprilTagDetection tagOfInterest;
//
//    int park = 0;
//    int stack = 1;
//    double jx = -30.75, jy = -2.75;
//    CRobot robot = new CRobot();
//    double sx = -40, sy = -10.7, sK = 22;//bun -> sx = -45 sy = -11.5
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcamName = hardwareMap.get(WebcamName.class, "camera");
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//                // camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(100);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        // -24 0 11 11
//        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
//        drive.setPoseEstimate(startPose);
//
//        robot.init(telemetry, hardwareMap);
//        while (!isStarted() && !isStopRequested()) {
//
//            ArrayList<AprilTagDetection> currentDetections = parkTag.getLatestDetections();
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == ID_TAG_OF_INTEREST_0 || tag.id == ID_TAG_OF_INTEREST_1 || tag.id == ID_TAG_OF_INTEREST_2) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//                if (tagFound) {
//                    telemetry.addLine("Tag of interest sighted\n\nLocationData:");
//                    tagToTelemetry(tagOfInterest);
//                } else {
//                    telemetry.addLine("Don't see tag of interest");
//                    if (tagOfInterest == null) {
//                        telemetry.addLine("(The tag has never been seen)");
//                    } else {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//                switch (tagOfInterest.id) {
//                    case 0:
//                        park = 1;
//                        break;
//                    case 9:
//                        park = 2;
//                        break;
//                    case 19:
//                        park = 3;
//                        break;
//                }
//            } else {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if (tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                sleep(10);
//            }
//            telemetry.update();
//
//        }

//        }
//    private void initTfod() {
//
//        // Create the TensorFlow processor by using a builder.
//        tfod = new TfodProcessor.Builder()
//
//                // With the following lines commented out, the default TfodProcessor Builder
//                // will load the default model for the season. To define a custom model to load,
//                // choose one of the following:
//                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
//                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                //.setModelAssetName(TFOD_MODEL_ASSET)
//                //.setModelFileName(TFOD_MODEL_FILE)
//
//                // The following default settings are available to un-comment and edit as needed to
//                // set parameters for custom models.
//                //.setModelLabels(LABELS)
//                //.setIsModelTensorFlow2(true)
//                //.setIsModelQuantized(true)
//                //.setModelInputSize(300)
//                //.setModelAspectRatio(16.0 / 9.0)
//
//                .build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        //builder.enableLiveView(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        //builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor.
//        builder.addProcessor(tfod);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Set confidence threshold for TFOD recognitions, at any time.
//        //tfod.setMinResultConfidence(0.75f);
//
//        // Disable or re-enable the TFOD processor at any time.
//        //visionPortal.setProcessorEnabled(tfod, true);
//
//    }
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
//    // this is only used for Android Studio when using models in Assets.
//    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
//    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
//    // this is used when uploading models directly to the RC using the model upload interface.
//    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
//    // Define the labels recognized in the model for TFOD (must be in training order!)
//    private static final String[] LABELS = {
//            "Pixel",
//    };
//
//    /**
//     * The variable to store our instance of the TensorFlow Object Detection processor.
//     */
//    private TfodProcessor tfod;
//
//    /**
//     * The variable to store our instance of the vision portal.
//     */
//    private VisionPortal visionPortal;
//    }
//
//    void tagToTelemetry(AprilTagDetection detection) {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("\nParking Spot = %d", park));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//
//    }
//}
///*
//Salut ! Mă numesc Luca si fac parte din departamentul de programare. La fel ca si corpul robotului, creierul acestuia, adica programele folosite in insufletirea acestui s-au schimbat si adaptat la fel de mult.
//
//Pentru cod, am folosit aplicatia Android Studio, in timp ce in anii trecuti foloseam OnBotJava. Aceasta alegere a fost motivata de diversele imbunatatiri pe care le aduce Android Studio, precum integrarea GitHub ului ce ne a ajutat enorm in fluidizarea procesului de coding.
//
//Pentru TeleOp, am creat un cod in care am segmentat fiecare actiune a robotului in cate o functie. Acest lucru ne a permis sa ne focusam pe asigurarea si imbunatatirea controlului driverilor fara a ne complica prea mult cu structurarea. Pe parcursul realizarii codului, am comunicat cu driverii, unul dintre ei fiind eu, pentru a face dirijarea robotului o munca cat mai usoara si intuitiva.
//
//Pentru autonomie, folosind modulele de odometrie si libraria roadrunner am dat viata robotului in acele 30 de secunde de aur. Procesul realizarii codului pentru autonomie a fost unul foarte amanuntit, bazat pe mai multe etape. In primul rand, am gandit o strategie de joc a robotului, bazata pe capabilitatiile robotului. Dupa aceea, am trecut strategia respectiva prin libraria MeepMeep, cu ajutorul careia am putut analiza o simulare aproximativa a  robotului pe teren. In final, am aplicat si am modificat codul final pe robotul fizic.
//
//*/
// /*
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
//        drive.setPoseEstimate(startPose);
//        Trajectory trajSignalDisplacement = drive.trajectoryBuilder(startPose)
//                .addDisplacementMarker(() -> {
//                    robot.runLift(11);
//                })
//                .forward(55)
//                .build();
//        Trajectory trajSignalReposition = drive.trajectoryBuilder(trajSignalDisplacement.end())
//                .back(10)
//                .build();
//        Trajectory trajFirstCap = drive.trajectoryBuilder(trajSignalReposition.end())
//                .splineTo(new Vector2d(-28, -2.5), Math.toRadians(45))// lasat con
//                //.forward(10)
//                .build();
//        //Variant 2
//        Trajectory FirstCapRepoV3 = drive.trajectoryBuilder(trajFirstCap.end())
//                .lineToSplineHeading(new Pose2d(-35,-10,Math.toRadians(180)))
//                .addDisplacementMarker(() -> {
//                    sleep(100);
//                    robot.runLiftStack(0);
//                })
//                //.lineToSplineHeading(new Pose2d(-35,-15,Math.toRadians(90)))
//                .build();
//        Trajectory ConeStackV3 = drive.trajectoryBuilder(FirstCapRepoV3.end())
//                //.splineTo(new Vector2d(-35,-9.5),Math.toRadians(180))
//                .forward(21)
//                .build();
//        Trajectory ConeStackRetractV3 = drive.trajectoryBuilder(ConeStackV3.end())
//                .addDisplacementMarker(() -> {
//            robot.runLift(11);
//        })
//                .back(20)
//                .build();
//        Trajectory ConeStackRepoV3= drive.trajectoryBuilder(ConeStackRetractV3.end())
//                .lineToSplineHeading(new Pose2d(-35,-15,Math.toRadians(90)))
//                .build();
//        Trajectory SecondCapV3 = drive.trajectoryBuilder(ConeStackRetractV3.end())
//                .splineTo(new Vector2d(-29, -3.5), Math.toRadians(45))// lasat con
//                .build();
//        Trajectory SecondCapRepoV3 = drive.trajectoryBuilder(SecondCapV3.end())
//                .lineToSplineHeading(new Pose2d(-35,-9.5,Math.toRadians(180)))
//                .addDisplacementMarker(() -> {
//                    sleep(100);
//                    robot.runLiftStack(0);
//                })
//                //.lineToSplineHeading(new Pose2d(-35,-15,Math.toRadians(90)))
//                .build();
//        Trajectory ConeStackSecondV3 = drive.trajectoryBuilder(SecondCapRepoV3.end())
//                //.splineTo(new Vector2d(-35,-9.5),Math.toRadians(180))
//                .forward(21)
//                .build();
//        Trajectory ConeStackSecondRetractV3 = drive.trajectoryBuilder(ConeStackSecondV3.end())
//                .addDisplacementMarker(() -> {
//                    robot.runLift(11);
//                })
//                .back(20)
//                .build();
//        Trajectory ThirdCapV3 = drive.trajectoryBuilder(ConeStackRetractV3.end())
//                .splineTo(new Vector2d(-29, -3.5), Math.toRadians(45))// lasat con
//                .build();
//        //
//        Trajectory trajFirstCapReposition = drive.trajectoryBuilder(trajFirstCap.end())
//                .back(14)
//                .build();
//        Trajectory trajConeStack = drive.trajectoryBuilder(trajFirstCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(135))))
//                .forward(27)
//                .build();
//        //Variant
//        Trajectory trajConeStackRepositionV2 = drive.trajectoryBuilder(ConeStackV3.end())
//                .addDisplacementMarker(() -> {
//                    robot.runLift(11);
//                })
//                .back(50)
//                .build();
//        Trajectory trajSecondCapV2 = drive.trajectoryBuilder(trajConeStackRepositionV2.end())
//                .splineTo(new Vector2d(-22, -5), Math.toRadians(135))
//                .build();
//        Trajectory trajSecondCapRepositionV2 = drive.trajectoryBuilder(trajSecondCapV2.end())
//                .back(16).build();
//        Trajectory trajConeStack2V2 = drive.trajectoryBuilder(trajSecondCapRepositionV2.end())
//                .splineTo(new Vector2d(-27,trajConeStack.end().getY()),Math.toRadians(180))
//                .addDisplacementMarker(() -> {
//                    robot.runLiftStack(0);
//                })
//                .forward(-trajConeStack.end().getX()-27)
//                .build();
//        Trajectory trajConeStackReposition2V2 = drive.trajectoryBuilder(trajConeStack2V2.end())
//                .addDisplacementMarker(() -> {
//                    robot.runLift(11);
//                })
//                .back(50)
//                .build();
//        Trajectory trajThirdCapV2 = drive.trajectoryBuilder(trajConeStackReposition2V2.end())
//                .splineTo(new Vector2d(-21, -5), Math.toRadians(135))
//                .build();
//        //
//        Trajectory trajConeStackReposition = drive.trajectoryBuilder(trajConeStack.end())
//                .back(23)
//                .addDisplacementMarker(() -> {
//                    robot.runLift(11);
//                })
//                .build();
//        Trajectory trajSecondCap = drive.trajectoryBuilder(trajConeStackReposition.end().plus(new Pose2d(0, 0, Math.toRadians(-135)))).
//                forward(13)
//                .build();
//        Trajectory trajSecondCapReposition = drive.trajectoryBuilder(trajSecondCap.end())
//                .back(13).build();
//        Trajectory trajConeStack2 = drive.trajectoryBuilder(trajSecondCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(135))))
//                .forward(23)
//                .build();
//
//        Trajectory trajp0 = drive.trajectoryBuilder(trajSecondCapReposition.end())
//                .back(1)
//                .build();
//        Trajectory trajp1 = drive.trajectoryBuilder(trajSecondCapRepositionV2.end())
//                .splineTo(new Vector2d(-27,trajConeStack.end().getY()),Math.toRadians(135))
//                .build();
//        Trajectory trajp2 = drive.trajectoryBuilder(trajSecondCapRepositionV2.end())
//                .splineTo(new Vector2d(-13,trajConeStack.end().getY()),Math.toRadians(135))
//                .build();
//        Trajectory trajp3 = drive.trajectoryBuilder(trajSecondCapRepositionV2.end())
//                .back(1)
//                .build();
//                */
// /* while(!tagFound && !opModeIsActive()) {
//       if(currentDetections.size() !=0)
//           for(AprilTagDetection tag : currentDetections)
//           {
//               if(tag.id ==1){tagFound=true;park=1;}
//                   else
//               if(tag.id == 10){tagFound=true;park=2;}
//                   else
//               if(tag.id == 19){tagFound=true;park=3;}
//                   else park=0;
//           }
//           telemetry.addData("park detection", park);
//       }*/