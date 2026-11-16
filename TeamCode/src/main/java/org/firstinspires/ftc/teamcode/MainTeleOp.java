package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Port map, directions and encoders: see PORTS.md in the project folder.
 *
 * CONTROLS (gamepad 1)
 *   left stick   drive and strafe      right stick X   turn
 *   L2 intake    L1 reverse intake     R2 shoot        R1 reverse shooter
 *   A flower bar down                  Y flower bar up
 */
@TeleOp(name = "YusufTele", group = "Competition")
public class MainTeleOp extends OpMode {

    static final double TRANSFER_POWER = 1.0;
    static final double SHOOTER_POWER = 1.0;
    static final double STRAFE_FIX = 1.1;   // strafing is a bit slower than driving forward, so boost it

    // Servo positions go from 0.0 to 1.0. These are guesses: test with the robot on a stand.
    static final double FLOWER_UP = 1;
    static final double FLOWER_DOWN = 0.45;   // TUNE: was 0.3 (too low). Bigger = bar stops higher.

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor transferLeft, transferRight, shooterLeft, shooterRight;
    Servo flower1, flower2;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        transferLeft = hardwareMap.get(DcMotor.class, "tl");
        transferRight = hardwareMap.get(DcMotor.class, "tr");
        shooterLeft = hardwareMap.get(DcMotor.class, "sl");
        shooterRight = hardwareMap.get(DcMotor.class, "sr");
        flower1 = hardwareMap.get(Servo.class, "f1");
        flower2 = hardwareMap.get(Servo.class, "f2");

        // On this robot the RIGHT wheels are reversed (most robots reverse the left).
        // Found by testing: with this, the normal mecanum math below works for drive, strafe and turn.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Each pair faces each other, so one of each pair is reversed.
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // The two flower servos are mirror images, so one is reversed.
        // Then the same position number moves both the same way.
        flower2.setDirection(Servo.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Ready. Press START.");
    }

    @Override
    public void loop() {
        // ----- Drive -----
        // Pushing a stick forward gives a NEGATIVE y, so flip it.
        double y = -deadband(gamepad1.left_stick_y);
        double x = deadband(gamepad1.left_stick_x) * STRAFE_FIX;
        double turn = deadband(gamepad1.right_stick_x);

        // Divide so no wheel is asked for more than 1.0 (full power).
        double biggest = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1.0);
        frontLeft.setPower((y + x + turn) / biggest);
        backLeft.setPower((y - x + turn) / biggest);
        frontRight.setPower((y - x - turn) / biggest);
        backRight.setPower((y + x - turn) / biggest);

        // ----- Intake / transfer -----
        if (gamepad1.left_bumper) {
            setTransfer(-TRANSFER_POWER);
        } else if (gamepad1.left_trigger > 0.1) {
            setTransfer(TRANSFER_POWER);
        } else {
            setTransfer(0);
        }

        // ----- Shooter -----
        if (gamepad1.right_bumper) {
            setShooter(-SHOOTER_POWER);
        } else if (gamepad1.right_trigger > 0.1) {
            setShooter(SHOOTER_POWER);
        } else {
            setShooter(0);
        }

        // ----- Flower bar -----
        // A servo holds whatever position it was last told, so we only send it when a button is pressed.
        if (gamepad1.a) {
            setFlower(FLOWER_DOWN);
        } else if (gamepad1.y) {
            setFlower(FLOWER_UP);
        }

        telemetry.addData("Flower position", flower1.getPosition());
    }

    double deadband(double value) {
        if (Math.abs(value) < 0.08) {
            return 0;
        }
        return value;
    }

    void setTransfer(double power) {
        transferLeft.setPower(power);
        transferRight.setPower(power);
    }

    void setShooter(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    void setFlower(double position) {
        flower1.setPosition(position);
        flower2.setPosition(position);
    }
}
