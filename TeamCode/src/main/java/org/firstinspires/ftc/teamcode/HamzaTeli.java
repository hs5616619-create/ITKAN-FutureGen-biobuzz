package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "HamzaTeli", group = "Competition")
public class HamzaTeli extends OpMode {


    private static final double TRANSFER_POWER = -1.0;
    private static final double REVERSE_POWER  = 1.0;


    private static final double TARGET_SHOOTER_VELOCITY = 2000.0; // Ticks per second
    private static final double SHOOTER_IDLE_POWER      = 0.2;    // Maintenance power


    private static final double FLOWER_UP_POS   = 1.0;
    private static final double FLOWER_DOWN_POS = 0.6;


    private static final double STOPPER_CLOSED_POS = 0.0;
    private static final double STOPPER_OPEN_POS   = 0.5;
    private static final double FEED_DURATION_MS   = 300.0; // Time in ms to stay open for 1 pollen

    private static final boolean SWAP_BACK_MOTORS = false;


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor transferLeft, transferRight;
    private DcMotorEx shooterLeft, shooterRight;
    private Servo flowerServo1, flowerServo2;
    private Servo stopperServo;

    private boolean flowerIsDown = false;


    private ElapsedTime feedTimer = new ElapsedTime();
    private boolean isFeeding = false;

    @Override
    public void init() {

        frontLeft    = hardwareMap.get(DcMotor.class, "fl");
        frontRight   = hardwareMap.get(DcMotor.class, "fr");
        backLeft     = hardwareMap.get(DcMotor.class, SWAP_BACK_MOTORS ? "br" : "bl");
        backRight    = hardwareMap.get(DcMotor.class, SWAP_BACK_MOTORS ? "bl" : "br");
        transferLeft  = hardwareMap.get(DcMotor.class, "tl");
        transferRight = hardwareMap.get(DcMotor.class, "tr");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "sl");
        shooterRight = hardwareMap.get(DcMotorEx.class, "sr");
        flowerServo1 = hardwareMap.get(Servo.class, "f1");
        flowerServo2 = hardwareMap.get(Servo.class, "f2");
        stopperServo = hardwareMap.get(Servo.class, "ss");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRight.setDirection(DcMotorSimple.Direction.FORWARD);

        flowerServo1.setDirection(Servo.Direction.FORWARD);
        flowerServo2.setDirection(Servo.Direction.REVERSE);
        stopperServo.setDirection(Servo.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        setFlower(FLOWER_UP_POS);
        stopperServo.setPosition(STOPPER_CLOSED_POS);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();
    }

    @Override
    public void loop() {

        double y  = deadband(gamepad1.left_stick_y);
        double x  = deadband(-gamepad1.left_stick_x) * 1.1;
        double rx = deadband(gamepad1.right_stick_x);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);


        if (gamepad1.left_bumper) {
            setTransfer(REVERSE_POWER);
        } else if (gamepad1.left_trigger > 0.1) {
            setTransfer(TRANSFER_POWER);
        } else {
            setTransfer(0);
        }


        if (gamepad1.cross) {
            flowerIsDown = true;
        } else if (gamepad1.triangle) {
            flowerIsDown = false;
        }
        setFlower(flowerIsDown ? FLOWER_DOWN_POS : FLOWER_UP_POS);


        if (gamepad1.right_bumper) {

            setShooterPower(-1.0);
            stopperServo.setPosition(STOPPER_CLOSED_POS);
            isFeeding = false;
        } else if (gamepad1.right_trigger > 0.1) {

            runBangBangShooter();
            runStopperLogic();
        } else {

            setShooterPower(0);
            stopperServo.setPosition(STOPPER_CLOSED_POS);
            isFeeding = false;
        }


        telemetry.addData("Shooter L Vel", shooterLeft.getVelocity());
        telemetry.addData("Shooter R Vel", shooterRight.getVelocity());
        telemetry.addData("Stopper State", isFeeding ? "OPEN (Feeding)" : "CLOSED");
        telemetry.addData("Flower State", flowerIsDown ? "DOWN" : "UP");
        telemetry.update();
    }



    private void runBangBangShooter() {
        double currentVelocity = (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;

        if (currentVelocity < TARGET_SHOOTER_VELOCITY) {
            setShooterPower(1.0); // recover
        } else {
            setShooterPower(SHOOTER_IDLE_POWER); // maintain
        }
    }

    private void runStopperLogic() {
        double currentVelocity = (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;


        if (currentVelocity >= TARGET_SHOOTER_VELOCITY) {
            if (!isFeeding) {
                stopperServo.setPosition(STOPPER_OPEN_POS);
                feedTimer.reset();
                isFeeding = true;
            }
        }


        if (isFeeding && feedTimer.milliseconds() >= FEED_DURATION_MS) {
            stopperServo.setPosition(STOPPER_CLOSED_POS);
            isFeeding = false;
        }
    }

    private double deadband(double v) {
        return Math.abs(v) < 0.08 ? 0 : v;
    }

    private void setTransfer(double power) {
        transferLeft.setPower(power);
        transferRight.setPower(power);
    }

    private void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    private void setFlower(double position) {
        flowerServo1.setPosition(position);
        flowerServo2.setPosition(position);
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);}
}
