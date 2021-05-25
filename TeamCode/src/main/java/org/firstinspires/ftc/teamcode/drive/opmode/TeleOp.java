package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "! LETS FRICKIN GO !")
public class TeleOp extends LinearOpMode {

    boolean canShoot, toShoot, toCollect, toPowerShoot, toSnatch, toZoom, toSnail, lastResetState, curResetState, lastResetState2, curResetState2, lastResetState3, curResetState3, lastResetState4, curResetState4, lastResetState5, curResetState5, lastResetState6, curResetState6 = false;
    boolean isShooting, wasShooting, isTripleShooting, wasTripleShooting = false;

    SampleMecanumDrive drive;

    DcMotorEx collector;
    Servo launch, ramp, snatch;
    public static double tPower = 0.68;
    public static double tOffset = 0.027;
    public static double sGrab = 0;
    public static double sRelease = 1.0;
    double v, prevV;
    long prevTimer;
    public static double dpad_speed = 0.4;
    public static double turn_speed = 0.3;
    public static double DRIVE_TRANS_SPEED = 0.65;
    public static double DRIVE_ROT_SPEED = 0.75;
    public static double DRIVE_SLOW_TRANS_SPEED = 0.45;
    public static double DRIVE_SLOW_ROT_SPEED = 0.35;
    public static double TRANS_SPEED = DRIVE_TRANS_SPEED;
    public static double ROT_SPEED = DRIVE_ROT_SPEED;
    public static int SHOOT_POWER = 1800;
    public static int SHOOT_DELAY = 800;
    public static int SERVO_DELAY = 400;
    public static double COLLECTOR_POWER = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        int soundPOW = hardwareMap.appContext.getResources().getIdentifier("pow", "raw", hardwareMap.appContext.getPackageName());
        int soundLETSGO = hardwareMap.appContext.getResources().getIdentifier("letsgo",   "raw", hardwareMap.appContext.getPackageName());
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collector = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        launch = hardwareMap.get(Servo.class, "launch");
        ramp = hardwareMap.get(Servo.class, "ramp");
        snatch = hardwareMap.get(Servo.class, "snatch");

        waitForStart();

        prevTimer = System.currentTimeMillis();
        while (opModeIsActive()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*TRANS_SPEED,
                            -gamepad1.left_stick_x*TRANS_SPEED,
                            -gamepad1.right_stick_x*ROT_SPEED
                    )
            );

            if (gamepad1.dpad_down) {
                drive.setWeightedDrivePower( new Pose2d(-dpad_speed, 0, 0) );
            }
            else if (gamepad1.dpad_up) {
                drive.setWeightedDrivePower( new Pose2d(dpad_speed, 0, 0) );
            }
            else if (gamepad1.dpad_left) {
                drive.setWeightedDrivePower( new Pose2d(0, dpad_speed, 0) );
            }
            else if (gamepad1.dpad_right) {
                drive.setWeightedDrivePower( new Pose2d(0, -dpad_speed, 0) );
            }
            else if (gamepad1.left_trigger > 0) {
                drive.setWeightedDrivePower( new Pose2d(0, 0, gamepad1.left_trigger * turn_speed) );
            }
            else if (gamepad1.right_trigger > 0) {
                drive.setWeightedDrivePower( new Pose2d(0, 0, -gamepad1.right_trigger * turn_speed) );
            }
            v = drive.shooter.getVelocity();

            canShoot = prevV == v && v == SHOOT_POWER;

            if (gamepad2.right_trigger > 0.1) {
                isShooting = true;
                launch.setPosition(0);
                prevTimer = System.currentTimeMillis();
                if (isShooting && !wasShooting) {
//                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundPOW);
                }
            } else {
                launch.setPosition(0.8);
                isShooting = false;
            }

            if (gamepad2.right_bumper) {
                isTripleShooting = true;
                if (isTripleShooting && !wasTripleShooting) {
//                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundLETSGO);
                }
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
            } else {
                isTripleShooting = false;
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

            drive.wobble.setPower(-gamepad2.left_stick_y*0.5);

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

            curResetState4 = (gamepad2.left_bumper);
            if (curResetState4 && !lastResetState4) {
                toSnatch = !toSnatch;
            }
            lastResetState4 = curResetState4;
            if (toSnatch) {
                snatch.setPosition(sGrab);
            } else {
                snatch.setPosition(sRelease);
            }

            curResetState5 = (gamepad1.b);
            if (curResetState5 && !lastResetState5) {
                toZoom = !toZoom;
            }
            lastResetState5 = curResetState5;
            if (toZoom) {
                TRANS_SPEED = 1;
                ROT_SPEED = 1;
            } else {
                TRANS_SPEED = DRIVE_TRANS_SPEED;
                ROT_SPEED = DRIVE_ROT_SPEED;
            }

            curResetState6 = (gamepad1.x);
            if (curResetState6 && !lastResetState6) {
                toSnail = !toSnail;
            }
            lastResetState6 = curResetState6;
            if (toSnail) {
                TRANS_SPEED = DRIVE_SLOW_TRANS_SPEED;
                ROT_SPEED = DRIVE_SLOW_ROT_SPEED;
            } else {
                TRANS_SPEED = DRIVE_TRANS_SPEED;
                ROT_SPEED = DRIVE_ROT_SPEED;
            }

            prevV = v;
            telemetry.addData("SHOOT?", canShoot ? "YES" : "NO");
            telemetry.addData("RAMP POS:", ramp.getPosition() == tPower ? "POWER" : "HIGH GOAL");
            telemetry.addData("SPEED:", TRANS_SPEED == DRIVE_TRANS_SPEED ? "NORMAL SPEED" : "NOT NORMAL SPEED");
            telemetry.update();
            drive.update();
//            SoundPlayer.getInstance().stopPlayingAll();

        }

    }

}
