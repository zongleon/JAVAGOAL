/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "? LETS FRICKIN GO ?")
public class Autonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private Servo launch, ramp, snatch;

    private SampleMecanumDrive drive;
    private String zone = "UNKNOWN";

    public static double ZOOM = 4;
    public static double RAMP_LAUNCH = 0.677;
    public static double START_X = -64;
    public static double START_Y = 48;
    Pose2d startPose = new Pose2d(START_X, START_Y, 0);
    public static double A_X = 18;
    public static double A_Y = 61;
    public static double B1_X = -24;
    public static double B1_Y = 60;
    public static double B2_X = 16;
    public static double B2_Y = 52;
    public static double conf = 0.8f;
    public static double B4_C4_X = 0;
    public static double B4_C4_Y = 18;
    public static double C1_X = 0;
    public static double C1_Y = 58;
    public static double C2_X = 40;
    public static double C2_Y = 50;
    public static double F_OFFSET_X = -4;
    public static double F_OFFSET_Y = -4;
    public static double R_X = -37;
    public static double R_Y = 20;
    public static double SNATCH_GRAB = 0;
    public static double SNATCH_RELEASE = 1.0;
    public static double SHOOT_X = -5;
    public static double SHOOT_Y = 30;
    public static double SHOOT_THETA_A = 0;
    public static double SHOOT_THETA_B = -9;
    public static double SHOOT_THETA_C = -7;
    public static double PARK_X = 10;
    public static double PARK_Y = 35;
    public static double SHOOT_DELTA = 6.5;
    public static double wobblePower = 0.51;
    public static int WOBBLE_DELAY = 650;
    public static double WOBBLE_Y_OFFSET_A = 3;
    public static double WOBBLE_Y_OFFSET_B = 2;
    public static double WOBBLE_Y_OFFSET_C = 1;
    public static int SHOOT_OFFSET = 850;
    public static int ACTION_DELAY = 600;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AdQfAyr/////AAABmUh6Z5KT20+RoULUpgxmoc9nIV2FKHL5EaGvj3PPgHtOujprWlIvVPgxtFaYImMYo175bgHUe+tHxxYynQmrgtrPcCBOIgpyptC6DCkr4lG4jZ59rDYEVPh+IUNKMWOgtphivaS+ZSclNCN2+uE40/oqQ0HuRLAGcxe/UviDbt6IafV2RkFFs412uP1E5XL/66hm46TahtlARJNQsKMTrxCNa8OFwvzC9ZW/ryimTGl46MdL9L6oI8JLHGm7GB7y7GS9GtqasKZvhgP4QCNgKHUDiC6urJ2BML9DO34qRY9zEELLG1fi92G4tB7P/0BsREjvNs28UNrXYrldXaJkAIK3pK2NJHNWFUuy1h7mgd+x";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        int soundA = hardwareMap.appContext.getResources().getIdentifier("zonea", "raw", hardwareMap.appContext.getPackageName());
        int soundB = hardwareMap.appContext.getResources().getIdentifier("zoneb",   "raw", hardwareMap.appContext.getPackageName());
        int soundC = hardwareMap.appContext.getResources().getIdentifier("zonec", "raw", hardwareMap.appContext.getPackageName());
        launch = hardwareMap.get(Servo.class, "launch");
        ramp = hardwareMap.get(Servo.class, "ramp");
        snatch = hardwareMap.get(Servo.class, "snatch");
        launch.setPosition(0.8);
        ramp.setPosition(RAMP_LAUNCH);
        snatch.setPosition(SNATCH_GRAB);
        drive = new SampleMecanumDrive(hardwareMap);
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(ZOOM, 16.0/9.0);
        }

        /** Wait for the game to begin */
        long start = System.currentTimeMillis();
        while(!isStopRequested() && zone.equals("UNKNOWN")) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0 && System.currentTimeMillis() > start + 4000) {
                        // empty list.  no objects recognized.
                        zone = "A";
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                zone = "B";
                                telemetry.addData("Target Zone", "B");
                            } else if (recognition.getLabel().equals("Quad")) {
                                zone = "C";
                                telemetry.addData("Target Zone", "C");
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                            }
                        }
                    }

                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("Data>",zone);
        telemetry.update();

        waitForStart();

        // (0,0) is bottom left corner
        drive.setPoseEstimate(startPose);
        switch(zone) {
            case "A":

                Trajectory traj1a = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(A_X, A_Y), 0)
                        .build();
                Trajectory traj2a = drive.trajectoryBuilder(traj1a.end(), Math.toRadians(-90))
                        .addDisplacementMarker(this::windup)
                        .splineToLinearHeading(new Pose2d(SHOOT_X, SHOOT_Y,Math.toRadians(SHOOT_THETA_A)), Math.toRadians(-90))
                        .build();

                Trajectory traj3a = drive.trajectoryBuilder(new Pose2d(SHOOT_X, SHOOT_Y - SHOOT_DELTA - SHOOT_DELTA), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(R_X, R_Y + WOBBLE_Y_OFFSET_A), Math.toRadians(180))
                        .build();
                Trajectory traj4a = drive.trajectoryBuilder(new Pose2d(R_X, R_Y + WOBBLE_Y_OFFSET_A), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(A_X + F_OFFSET_X, A_Y + F_OFFSET_Y), Math.toRadians(0))
                        .build();
                Trajectory traj5a = drive.trajectoryBuilder(traj4a.end(), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(PARK_X, PARK_Y), Math.toRadians(-90))
                        .build();

                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundA);
                drive.followTrajectory(traj1a);
                deliver();
                drive.followTrajectory(traj2a);
                shoot();
                drive.followTrajectory(traj3a);
                grab();
                drive.followTrajectory(traj4a);
                deliver2();
                drive.followTrajectory(traj5a);

                break;
            case "B":
                Trajectory traj1b = drive.trajectoryBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(B1_X, B1_Y), 0)
                        .splineToSplineHeading(new Pose2d(B2_X,B2_Y,Math.toRadians(-179)),Math.toRadians(-90))
                        .build();
                Trajectory traj2b = drive.trajectoryBuilder(traj1b.end())
                        .addDisplacementMarker(this::windup)
                        .splineToLinearHeading(new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_THETA_B)), Math.toRadians(-90))
                        .build();

                Trajectory traj3b = drive.trajectoryBuilder(new Pose2d(SHOOT_X, SHOOT_Y - SHOOT_DELTA - SHOOT_DELTA), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(R_X, R_Y + WOBBLE_Y_OFFSET_B), Math.toRadians(180))
                        .build();
                Trajectory traj4b = drive.trajectoryBuilder(new Pose2d(R_X, R_Y + WOBBLE_Y_OFFSET_B), Math.toRadians(0))
                        .splineTo(new Vector2d(B4_C4_X, B4_C4_Y), 0)
                        .splineToSplineHeading(new Pose2d(B2_X + F_OFFSET_X,B2_Y + F_OFFSET_Y, Math.toRadians(180)),Math.toRadians(90))
                        .build();
                Trajectory traj5b = drive.trajectoryBuilder(traj4b.end())
                        .splineToLinearHeading(new Pose2d(PARK_X, PARK_Y), Math.toRadians(-90))
                        .build();

                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundB);
                drive.followTrajectory(traj1b);
                deliver();
                drive.followTrajectory(traj2b);
                shoot();
                drive.followTrajectory(traj3b);
                grab();
                drive.followTrajectory(traj4b);
                deliver2();
                drive.followTrajectory(traj5b);

                break;
            case "C":
                Trajectory traj1c = drive.trajectoryBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(C1_X, C1_Y), 0)
                        .splineTo(new Vector2d(C2_X, C2_Y),-90)
                        .build();
                Trajectory traj2c = drive.trajectoryBuilder(traj1c.end())
                        .addDisplacementMarker(this::windup)
                        .splineToLinearHeading(new Pose2d(SHOOT_X, SHOOT_Y,Math.toRadians(SHOOT_THETA_C)), Math.toRadians(180))
                        .build();

                Trajectory traj3c = drive.trajectoryBuilder(new Pose2d(SHOOT_X, SHOOT_Y - SHOOT_DELTA - SHOOT_DELTA), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(R_X, R_Y + WOBBLE_Y_OFFSET_C), Math.toRadians(180))
                        .build();
                Trajectory traj4c = drive.trajectoryBuilder(new Pose2d(R_X, R_Y + WOBBLE_Y_OFFSET_C), Math.toRadians(0))
                        .splineTo(new Vector2d(B4_C4_X, B4_C4_Y), 0)
                        .splineToSplineHeading(new Pose2d(C2_X + F_OFFSET_X, C2_Y + F_OFFSET_Y, Math.toRadians(-90)),45)
                        .build();

                Trajectory traj5c = drive.trajectoryBuilder(traj4c.end())
                        .splineToLinearHeading(new Pose2d(PARK_X, PARK_Y, 0), Math.toRadians(180))
                        .build();

                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundC);
                drive.followTrajectory(traj1c);
                deliver();
                drive.followTrajectory(traj2c);
                shoot();
                drive.followTrajectory(traj3c);
                grab();
                drive.followTrajectory(traj4c);
                deliver2();
                drive.followTrajectory(traj5c);

                break;
            default:
                break;
        }


    }

    void deliver() {
        drive.wobble.setPower(-wobblePower);
        sleep(WOBBLE_DELAY);
        drive.wobble.setPower(0);
        snatch.setPosition(SNATCH_RELEASE);
        sleep(ACTION_DELAY);
        sleep(ACTION_DELAY);
    }

    void deliver2() {
        snatch.setPosition(SNATCH_RELEASE);
        sleep(ACTION_DELAY);
        drive.wobble.setPower(wobblePower);
        sleep(WOBBLE_DELAY);
        drive.wobble.setPower(0);
        sleep(ACTION_DELAY);
    }

    void windup() {
        drive.shooter.setVelocity(2580);
    }
    void shoot() {

//        drive.turn(0);

        Trajectory traj3a = drive.trajectoryBuilder(new Pose2d(SHOOT_X, SHOOT_Y), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(SHOOT_X, SHOOT_Y - SHOOT_DELTA), Math.toRadians(-90))
                .build();

        Trajectory traj3b = drive.trajectoryBuilder(traj3a.end(), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(SHOOT_X, SHOOT_Y - SHOOT_DELTA - SHOOT_DELTA), Math.toRadians(-90))
                .build();

        launch.setPosition(0);
        sleep(SHOOT_OFFSET);
        launch.setPosition(0.8);

        drive.followTrajectory(traj3a);
        sleep(SHOOT_OFFSET);

        launch.setPosition(0);
        sleep(SHOOT_OFFSET);
        launch.setPosition(0.8);

        drive.followTrajectory(traj3b);
        sleep(SHOOT_OFFSET);

        launch.setPosition(0);
        sleep(SHOOT_OFFSET);
        launch.setPosition(0.8);

        drive.shooter.setVelocity(0);
        sleep(ACTION_DELAY);

    }

    void grab() {
        snatch.setPosition(SNATCH_GRAB);
        sleep(ACTION_DELAY);
        drive.wobble.setPower(wobblePower);
        sleep(WOBBLE_DELAY);
        drive.wobble.setPower(0);
        sleep(ACTION_DELAY);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float) conf;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}