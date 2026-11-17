package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "PedroV2", group = "Autonomous")
public class PedroV2 extends LinearOpMode {

    private static final double TRANSFER_POWER = -1.0;

    private static final double TRANSFER_TIME_MS = 1500;//time of intaking

    private static final double REVERSE_POWER  = 1.0;


    private static final double TARGET_SHOOTER_VELOCITY = 2000.0; // Ticks per second
    private static final double SHOOTER_IDLE_POWER      = 0.2;    // Maintenance power


    private static final double FLOWER_UP_POS   = 1.0;
    private static final double FLOWER_DOWN_POS = 0.9;


    private static final double STOPPER_CLOSED_POS = 0.0;
    private static final double STOPPER_OPEN_POS   = 0.5;
    private static final double FEED_DURATION_MS   = 300.0; // Time in ms to stay open for 1 pollen

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor transferLeft, transferRight;
    private DcMotorEx shooterLeft, shooterRight;
    private Servo flowerServo1, flowerServo2;
    private Servo stopperServo;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose sTART = poseFactory.of(60, 9, 90);
    private final Pose path1 = poseFactory.of(59.7203, 13.9835, 270);
    private final Pose point2Start = poseFactory.of(59.7203, 13.9835, 270);
    private final Pose point2 = poseFactory.of(21.8519, 46.8937, 180);
    private final Pose point3Start = poseFactory.of(21.8519, 46.8937, 180);
    private final Pose point3 = poseFactory.of(15.5925, 46.8966, 180);
    private final Pose flower1toshotBStart = poseFactory.of(15.5925, 46.8966, 180);
    private final Pose flower1toshotB = poseFactory.of(63.621, 126.3525, 80);
    private final Pose flower1toshotBControl1 = poseFactory.of(14.4827, 107.2396, 0);
    private final Pose point5Start = poseFactory.of(63.621, 126.3525, 80);
    private final Pose point5 = poseFactory.of(46.6685, 118.7013, 90);
    private final Pose point6Start = poseFactory.of(46.6685, 118.7013, 90);
    private final Pose point6 = poseFactory.of(46.6287, 125.5548, 90);
    private final Pose point7Start = poseFactory.of(46.6287, 125.5548, 90);
    private final Pose point7 = poseFactory.of(63.8507, 126.3962, 80);
    private final Pose point8Start = poseFactory.of(63.8507, 126.3962, 80);
    private final Pose point8 = poseFactory.of(23.9616, 107.4503, 90);
    private final Pose point9Start = poseFactory.of(23.9616, 107.4503, 90);
    private final Pose point9 = poseFactory.of(10.8261, 107.2791, 90);


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                waitMs(2000),
                follow(follower, path2()),
                waitMs(750),
                follow(follower, path3()),
                waitMs(1500),
                follow(follower, flower1toshotB()),
                waitMs(2000),
                follow(follower, path5()),
                waitMs(750),
                follow(follower, path6()),
                waitMs(1500),
                follow(follower, path7()),
                waitMs(2000),
                follow(follower, path8()),
                follow(follower, path9())
        );
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

    Command transfer() {
        return sequential(
                instant(() -> setTransfer(TRANSFER_POWER)),
                waitMs(TRANSFER_TIME_MS),
                instant(() -> setTransfer(0))
        );
    }

    Command startCollecting() {
        return sequential(
                instant(() -> setFlower(FLOWER_DOWN_POS)),
                waitMs(750)
        );
    }




    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(sTART);
        follower.update();

        frontLeft    = hardwareMap.get(DcMotor.class, "fl");
        frontRight   = hardwareMap.get(DcMotor.class, "fr");
        backLeft     = hardwareMap.get(DcMotor.class, "bl");
        backRight    = hardwareMap.get(DcMotor.class, "br");
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


        //setFlower(FLOWER_UP_POS);
        stopperServo.setPosition(STOPPER_CLOSED_POS);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(sTART, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path flower1toshotB() {
        return Paths.curve(flower1toshotBStart, flower1toshotBControl1, flower1toshotB).linear(flower1toshotBStart, flower1toshotB);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.line(point9Start, point9).linear(point9Start, point9);
    }
}
