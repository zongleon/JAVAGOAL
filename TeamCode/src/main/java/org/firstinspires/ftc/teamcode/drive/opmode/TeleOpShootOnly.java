package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "! SHOOT ONLY !")
public class TeleOpShootOnly extends LinearOpMode {

    boolean canShoot, toShoot, toCollect, toPowerShoot, toSnatch, toZoom, toSnail, lastResetState, curResetState, lastResetState2, curResetState2, lastResetState3, curResetState3, lastResetState4, curResetState4, lastResetState5, curResetState5, lastResetState6, curResetState6 = false;
    boolean isShooting, wasShooting, isTripleShooting, wasTripleShooting = false;
    SampleMecanumDrive drive;

    DcMotorEx collector;

    Servo launch, ramp;
    public static double tPower = 0.68;
    public static double tOffset = 0.027;
    double v, prevV;

    public static boolean AUTO_SHOOT = false;
    public static int SHOOT_POWER = 2580;
    public static int SHOOT_DELAY = 800;
    public static int SERVO_DELAY = 400;
    public static double COLLECTOR_POWER = 0.75;

    public static double DRIVE_TRANS_SPEED = 0.65;
    public static double DRIVE_ROT_SPEED = 0.75;
    public static double TRANS_SPEED = DRIVE_TRANS_SPEED;
    public static double ROT_SPEED = DRIVE_ROT_SPEED;

    public static double START_X = 0;
    public static double START_Y = 0;
    private Pose2d START = new Pose2d(START_X, START_Y);
    public static double SHOOT_X = 0;
    public static double SHOOT_Y = -54;
    public static double SHOOT_T = 15;
    private Pose2d SHOOT = new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_T));

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collector = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        launch = hardwareMap.get(Servo.class, "launch");
        ramp = hardwareMap.get(Servo.class, "ramp");
        drive.setPoseEstimate(START);

        waitForStart();

        while (opModeIsActive()) {

            if (drive.isBusy() || gamepad1.left_stick_button) {
                drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.followTrajectory(makeTraj());
                drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                continue;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*TRANS_SPEED,
                            -gamepad1.left_stick_x*TRANS_SPEED,
                            -gamepad1.right_stick_x*ROT_SPEED
                    )
            );

            if (gamepad1.start) {
                drive.setPoseEstimate(START);
            }

            if (gamepad2.start) {
                SHOOT = drive.getPoseEstimate();
            }

            v = drive.shooter.getVelocity();

            if (gamepad2.right_trigger > 0.1) {
                drive.setPoseEstimate(SHOOT);
                launch.setPosition(0);
            } else {
                launch.setPosition(0.8);
            }

            if (gamepad2.right_bumper) {
                drive.setPoseEstimate(SHOOT);
                launch.setPosition(0);
                sleep(SERVO_DELAY);
                launch.setPosition(0.8);
                sleep(SHOOT_DELAY);
                launch.setPosition(0);
                sleep(SERVO_DELAY);
                launch.setPosition(0.8);
                sleep(SHOOT_DELAY);
                launch.setPosition(0);
                sleep(SERVO_DELAY);
                launch.setPosition(0.8);
            }

            curResetState = (gamepad2.a);
            if (curResetState && !lastResetState) {
                toShoot = !toShoot;
            }
            lastResetState = curResetState;
            if (toShoot) {
                drive.shooter.setVelocity(SHOOT_POWER);
            } else {
                drive.shooter.setVelocity(0);
            }
            telemetry.addData("LAUNCH vel:", v);

            curResetState3 = (gamepad2.x);
            if (curResetState3 && !lastResetState3) {
                toPowerShoot = !toPowerShoot;
            }
            lastResetState3 = curResetState3;
            if (toPowerShoot) {
                ramp.setPosition(tPower - tOffset);
            } else {
                ramp.setPosition(tPower);
            }

            curResetState2 = (gamepad2.b);
            if (curResetState2 && !lastResetState2) {
                toCollect = !toCollect;
            }
            lastResetState2 = curResetState2;
            if (toCollect && Math.abs(gamepad2.right_stick_y) < 0.15) {
                collector.setPower(-COLLECTOR_POWER);
            } else {
                collector.setPower(-gamepad2.right_stick_y);
            }

            prevV = v;
            telemetry.addData("SHOOT?", canShoot ? "YES" : "NO");
            telemetry.addData("RAMP POS:", ramp.getPosition() == tPower ? "POWER" : "HIGH GOAL");
            telemetry.update();
            drive.update();
//            SoundPlayer.getInstance().stopPlayingAll();

        }

    }

    Trajectory makeTraj() {
        Pose2d p1 = drive.getPoseEstimate();
        Pose2d p2 = SHOOT;
        double theta = Math.atan2(p2.getX() - p1.getX(), p2.getY() - p1.getY());
        telemetry.addData("THETA:", theta);

        return drive.trajectoryBuilder(p1, theta)
                .lineToSplineHeading(p2)
                .build();

    }

}
