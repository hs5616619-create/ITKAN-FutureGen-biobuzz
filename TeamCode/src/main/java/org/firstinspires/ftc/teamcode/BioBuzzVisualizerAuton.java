package org.firstinspires.ftc.teamcode;

/*
 * BIOBUZZ AUTON, MADE WITH THE PEDRO VISUALIZER  (red alliance; BioBuzzVisualizerAutonBlue is blue)
 *
 * HOW TO READ THIS FILE
 *   [EXPORTED]  = copied from the visualizer's export, unchanged.
 *   [ADDED]     = written by us (mechanisms, safety, telemetry).
 *   [CHANGED]   = an exported line we had to edit (the reason is next to it).
 *   VisualizerPathOnly.java is the export with nothing added, to compare against.
 *
 * TO CHANGE THE ROUTE
 *   Open visualizer/biobuzz-red-full.pp.json (in this project folder) in the visualizer (File > Open from computer),
 *   edit, then Export > Java code > Full code. Copy the [EXPORTED] pose lines and path methods over
 *   the ones below. Keep everything marked [ADDED]. If you rename or add paths, fix the follow(...)
 *   lines in autoRoutine() to match.
 *
 * WHAT IT DOES (about 22 s)
 *   Shoot the 4 preloads into cell A, collect from flower 1, drive around the hive, shoot into cell B,
 *   collect from flower 2, shoot into cell B again, park in the loading zone.
 *   No flywheel yet: "shoot" runs the intake long enough that the balls come out the back.
 *
 * JAVA NOTE: "() -> setIntake(0)" is a LAMBDA, a tiny function written inline. Ivy's instant(...)
 * takes one so it can run it LATER, when that step is reached, instead of right now.
 */

// ===== [EXPORTED] imports =====
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

// ===== [ADDED] imports =====
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;   // the export forgets this one
import static com.pedropathing.ivy.groups.Groups.race;   // runs steps side by side; done when the FIRST one finishes

// [CHANGED] name, group and preselectTeleOp (export: name = "AutoPath", group = "Autonomous")
@Autonomous(name = "BioBuzz Visualizer Auton (Red)", group = "Competition", preselectTeleOp = "YusufTele")
public class BioBuzzVisualizerAuton extends LinearOpMode {   // [CHANGED] class name (export: AutoPath)

    // ===== [ADDED] mechanism settings (TUNE these on the robot) =====
    static final double INTAKE_POWER = 1.0;
    static final double SHOOT_MS = 3000;        // long enough for every ball to come out the back
    static final double COLLECT_MS = 1500;      // long enough to pull balls in, short enough that they stay in
    static final double FLOWER_UP = 1.0;        // same positions as YusufTele
    static final double FLOWER_DOWN = 0.45;   // TUNE: was 0.3 (too low). Bigger = bar stops higher.
    static final double FLOWER_MOVE_MS = 400;   // time for the bar to finish moving
    static final double PUSH_TIMEOUT_MS = 1500; // give up pushing into a flower after this, even if not at the end point
    static final double PARK_DEADLINE_SECONDS = 25;   // stop everything and park if still busy at this time

    DcMotor transferLeft, transferRight;
    Servo flower1, flower2;
    ElapsedTime matchTimer = new ElapsedTime();

    // ===== [EXPORTED] follower and poses =====
    private Follower follower;

    // [CHANGED] export: "= PoseFactory.degrees();". makePoseFactory() lets the blue version rotate every pose.
    private final PoseFactory poseFactory = makePoseFactory();

    private final Pose start = poseFactory.of(60, 9, -90);
    private final Pose startToShotA = poseFactory.of(60, 22.6, -90);
    private final Pose shotAToFlower1Start = poseFactory.of(60, 22.6, -90);
    private final Pose shotAToFlower1 = poseFactory.of(26, 48, 180);
    private final Pose intoFlower1 = poseFactory.of(13.5, 48, 180);
    private final Pose flower1ToShotBStart = poseFactory.of(13.5, 48, 180);
    private final Pose flower1ToShotB = poseFactory.of(47.5, 119.4, 108);
    private final Pose flower1ToShotBControl1 = poseFactory.of(26, 95, 0);
    private final Pose shotBToFlower2Start = poseFactory.of(47.5, 119.4, 108);
    private final Pose shotBToFlower2 = poseFactory.of(48, 118, 90);
    private final Pose intoFlower2 = poseFactory.of(48, 128, 90);
    private final Pose flower2ToShotBStart = poseFactory.of(48, 128, 90);
    private final Pose flower2ToShotB = poseFactory.of(47.5, 119.4, 108);
    private final Pose shotBToParkStart = poseFactory.of(47.5, 119.4, 108);
    private final Pose shotBToPark = poseFactory.of(14, 108, 90);

    // [ADDED] Red uses the poses exactly as drawn. BioBuzzVisualizerAutonBlue overrides this.
    PoseFactory makePoseFactory() {
        return PoseFactory.degrees();
    }

    // ===== THE ROUTINE: [EXPORTED] follow(...) lines, with [ADDED] mechanism steps between them =====
    // sequential(...) runs each step after the previous one finishes.
    public Command autoRoutine() {
        return sequential(
            follow(follower, startToShotA()),
            shoot(),                                  // [ADDED] preloads into cell A

            follow(follower, shotAToFlower1()),
            startCollecting(),                        // [ADDED] flower bar down, intake on
            race(follow(follower, intoFlower1()), waitMs(PUSH_TIMEOUT_MS)),   // [CHANGED] export: follow(follower, intoFlower1()). Time limit in case the flower stops the robot short
            waitMs(COLLECT_MS),                       // [ADDED] give the pollen time to come in
            stopCollecting(),                         // [ADDED] intake off, flower bar up

            follow(follower, flower1ToShotB()),
            shoot(),                                  // [ADDED] into cell B

            follow(follower, shotBToFlower2()),
            startCollecting(),                        // [ADDED]
            race(follow(follower, intoFlower2()), waitMs(PUSH_TIMEOUT_MS)),   // [CHANGED] export: follow(follower, intoFlower2()). Time limit in case the flower stops the robot short
            waitMs(COLLECT_MS),                       // [ADDED]
            stopCollecting(),                         // [ADDED]

            follow(follower, flower2ToShotB()),
            shoot(),                                  // [ADDED] into cell B again

            follow(follower, shotBToPark())
        );
    }

    // ===== [ADDED] mechanism steps =====
    // Each method hands back a Command: a step that sequential(...) can run.

    // No flywheel yet: run the intake until the balls come out the back.
    Command shoot() {
        return sequential(
            instant(() -> setIntake(INTAKE_POWER)),
            waitMs(SHOOT_MS),
            instant(() -> setIntake(0))
        );
    }

    Command startCollecting() {
        return sequential(
            instant(() -> setFlower(FLOWER_DOWN)),
            instant(() -> setIntake(INTAKE_POWER)),
            waitMs(FLOWER_MOVE_MS)
        );
    }

    Command stopCollecting() {
        return sequential(
            instant(() -> setIntake(0)),
            instant(() -> setFlower(FLOWER_UP)),
            waitMs(FLOWER_MOVE_MS)
        );
    }

    void setIntake(double power) {
        transferLeft.setPower(power);
        transferRight.setPower(power);
    }

    void setFlower(double position) {
        flower1.setPosition(position);
        flower2.setPosition(position);
    }

    @Override
    public void runOpMode() {
        // [EXPORTED]
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // [ADDED] mechanism hardware: same names and directions as YusufTele
        transferLeft = hardwareMap.get(DcMotor.class, "tl");
        transferRight = hardwareMap.get(DcMotor.class, "tr");
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flower1 = hardwareMap.get(Servo.class, "f1");
        flower2 = hardwareMap.get(Servo.class, "f2");
        flower2.setDirection(Servo.Direction.REVERSE);
        telemetry.addLine("Place the robot at the start, intake facing the audience wall. Holding 4 pollen.");
        telemetry.update();

        // [EXPORTED]
        waitForStart();

        // [CHANGED] export: "schedule(autoRoutine());". We keep the routine in a variable so the
        // park deadline below can cancel it.
        matchTimer.reset();
        Command routine = autoRoutine();
        schedule(routine);
        boolean parking = false;

        while (opModeIsActive()) {
            // [EXPORTED]
            follower.update();
            Scheduler.execute();

            // [ADDED] safety: still busy at the deadline? Cancel the routine and park from wherever we are.
            if (!parking && Scheduler.isScheduled(routine) && matchTimer.seconds() > PARK_DEADLINE_SECONDS) {
                Scheduler.cancel(routine);
                setIntake(0);
                Pose here = follower.pose();
                schedule(follow(follower, Paths.line(here, shotBToPark).linear(here, shotBToPark)));
                parking = true;
            }

            // [CHANGED] telemetry: heading in degrees, plus time and parking status
            telemetry.addData("Time (s)", matchTimer.seconds());
            telemetry.addData("Parking early", parking);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }
        setIntake(0);   // [ADDED]
    }

    // ===== [EXPORTED] paths =====
    public Path startToShotA() {
        return Paths.line(start, startToShotA).linear(start, startToShotA);
    }

    public Path shotAToFlower1() {
        return Paths.line(shotAToFlower1Start, shotAToFlower1).linear(shotAToFlower1Start, shotAToFlower1);
    }

    public Path intoFlower1() {
        return Paths.line(shotAToFlower1, intoFlower1).constant(intoFlower1);
    }

    public Path flower1ToShotB() {
        return Paths.curve(flower1ToShotBStart, flower1ToShotBControl1, flower1ToShotB).linear(flower1ToShotBStart, flower1ToShotB);
    }

    public Path shotBToFlower2() {
        return Paths.line(shotBToFlower2Start, shotBToFlower2).linear(shotBToFlower2Start, shotBToFlower2);
    }

    public Path intoFlower2() {
        return Paths.line(shotBToFlower2, intoFlower2).constant(intoFlower2);
    }

    public Path flower2ToShotB() {
        return Paths.line(flower2ToShotBStart, flower2ToShotB).linear(flower2ToShotBStart, flower2ToShotB);
    }

    public Path shotBToPark() {
        return Paths.line(shotBToParkStart, shotBToPark).linear(shotBToParkStart, shotBToPark);
    }
}
