package org.firstinspires.ftc.teamcode.drive.opmode;

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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "! LETS FRICKIN GO !")
public class TeleOp extends LinearOpMode {

    //driving coefficients
    private double driveAxial = 0;   // Positive is forward
    private double driveLateral = 0;   // Positive is right
    private double driveYaw = 0;   // Positive is CCW

    boolean toShoot, toCollect, toPowerShoot, lastResetState, curResetState, lastResetState2, curResetState2, lastResetState3, curResetState3 = false;
    SampleMecanumDrive drive;

    DcMotorEx collector, shooter, wobble;
    Servo launch, ramp;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        collector = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobble = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        launch = hardwareMap.get(Servo.class, "launch");
        ramp = hardwareMap.get(Servo.class, "ramp");

        waitForStart();

        while (!isStopRequested()) {

            manualDrive(0.9);
            moveRobot();

            if (gamepad2.right_trigger > 0.1) {
                launch.setPosition(0);
            } else {
                launch.setPosition(0.8);
            }
            telemetry.addData("LAUNCH POS:", launch.getPosition());
            telemetry.addData("RAMP POS:", ramp.getPosition());

            curResetState = (gamepad2.a);
            if (curResetState && !lastResetState) {
                toShoot = !toShoot;
            }
            lastResetState = curResetState;
            if (toShoot) {
                shooter.setPower(1);
            } else {
                shooter.setPower(0);
            }

            curResetState2 = (gamepad2.b);
            if (curResetState2 && !lastResetState2) {
                toCollect = !toCollect;
            }
            lastResetState2 = curResetState2;
            if (toCollect) {
                collector.setPower(-0.6);
            } else {
                collector.setPower(-gamepad2.left_stick_y);
            }

            curResetState3 = (gamepad2.x);
            if (curResetState3 && !lastResetState3) {
                toPowerShoot = !toPowerShoot;
            }
            lastResetState3 = curResetState3;
            if (toPowerShoot) {
                ramp.setPosition(0.63);
            } else {
                ramp.setPosition(0.8);
            }

            telemetry.update();
        }

    }

    void manualDrive(double multiplier) {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        setAxial(-gamepad1.left_stick_y * multiplier);
        setLateral(gamepad1.left_stick_x * multiplier);
        setYaw(gamepad1.right_stick_x * multiplier);

    }

    void moveRobot() {
        // calculate required motor powers to achieve axis motions, based on the movement of mecanum wheels
        double backL = driveAxial - driveLateral + driveYaw;
        double backR = driveAxial + driveLateral - driveYaw;
        double left = driveAxial + driveLateral + driveYaw;
        double right = driveAxial - driveLateral - driveYaw;

        // normalize all motor speeds so no values exceeds 100%
        double max = Math.max(Math.abs(left), Math.abs(right));
        max = Math.max(max, Math.abs(backL));
        max = Math.max(max, Math.abs(backR));
        if (max > 1.0) {
            backR /= max;
            backL /= max;
            right /= max;
            left /= max;
        }

        // Set drive motor smartPower levels
        drive.leftRear.setPower(backL);
        drive.rightRear.setPower(backR);
        drive.leftFront.setPower(left);
        drive.rightFront.setPower(right);

        // Display Telemetry
        telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], >>Y[%+5.2f]<<", driveAxial, driveLateral, driveYaw);
        telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f], BL[%+5.2f], BR[%+5.2f]", left, right, backL, backR);

    }

    private void setAxial(double axial) {
        driveAxial = Range.clip(axial, -1, 1);
    }

    private void setLateral(double lateral) {
        driveLateral = Range.clip(lateral, -1, 1);
    }

    private void setYaw(double yaw) {
        driveYaw = Range.clip(yaw, -1, 1);
    }
}
