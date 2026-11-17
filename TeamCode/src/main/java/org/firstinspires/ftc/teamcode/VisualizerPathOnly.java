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
import org.firstinspires.ftc.teamcode.pedro.Constants;   // ADDED: the export forgets this import
// TODO 1 (imports): add imports for DcMotor, DcMotorSimple, Servo and ElapsedTime
//        (all in com.qualcomm.robotcore...), plus: import static com.pedropathing.ivy.groups.Groups.race;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.pedropathing.ivy.groups.Groups.race;

// this is exactly the code pedro visualizer exported (only added imports + changed class name)
//
// STUDENT PROJECT: copy this file, then turn the copy into a full auton with mechanisms by doing
// TODO 1 to TODO 9 in order. The "Auton Mechanisms Lab" page walks through each one.
// BioBuzzVisualizerAuton.java is the finished answer. Leave THIS file as the path-only test.
@Autonomous(name = "Visualizer Path Only (Red)", group = "Test")   // CHANGED: name
public class VisualizerPathOnly extends LinearOpMode {   // CHANGED: class name (must match the file name)

    // TODO 2 (settings): add "static final double" settings for the mechanisms:
    //        INTAKE_POWER, SHOOT_MS, COLLECT_MS, FLOWER_UP, FLOWER_DOWN, FLOWER_MOVE_MS, PUSH_TIMEOUT_MS,
    //        PARK_DEADLINE_SECONDS
    //        Use the same flower bar positions as YusufTele (FLOWER_UP = 1.0, FLOWER_DOWN = 0.45).

    // TODO 3a (hardware variables): declare the two transfer motors, the two flower servos,
    //        and an ElapsedTime match timer.

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

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

    // Autonomous routine
    // TODO 7 (routine): add mechanism steps between the follow(...) lines:
    //        shoot() after reaching each shooting spot, startCollecting() before each "into flower"
    //        path, then waitMs(COLLECT_MS) and stopCollecting() after it.
    // TODO 8 (push timeout): the "into flower" paths can get stuck pushing on the flower forever.
    //        Wrap each one: race(follow(...), waitMs(PUSH_TIMEOUT_MS))
    public Command autoRoutine() {
        return sequential(
            follow(follower, startToShotA()),
            follow(follower, shotAToFlower1()),
            follow(follower, intoFlower1()),
            follow(follower, flower1ToShotB()),
            follow(follower, shotBToFlower2()),
            follow(follower, intoFlower2()),
            follow(follower, flower2ToShotB()),
            follow(follower, shotBToPark())
        );
    }

    // TODO 4 (helpers): write setIntake(double power) and setFlower(double position).
    //        Each one sets BOTH motors / BOTH servos.
    // TODO 5 (shoot): write a method "Command shoot()" that runs the intake for SHOOT_MS, then stops it.
    //        Tools: sequential(...), instant(() -> ...), waitMs(...)
    // TODO 6 (collect): write "Command startCollecting()" (flower bar down, intake on, wait for the bar)
    //        and "Command stopCollecting()" (intake off, flower bar up, wait for the bar).

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // TODO 3b (hardware setup): get "tl", "tr", "f1", "f2" from hardwareMap and set the
        //        directions YusufTele uses (tl REVERSE, tr FORWARD, f2 REVERSE). See PORTS.md.

        waitForStart();
        // TODO 9 (park deadline): reset the match timer here. Keep the routine in a variable
        //        (Command routine = autoRoutine();) so the loop below can cancel it.
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // TODO 9 (continued): if the routine is still running after PARK_DEADLINE_SECONDS, cancel it,
            //        turn the intake off, and follow a straight line from here to the park pose.

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        // TODO 9 (end): turn the intake off when the OpMode stops.
    }

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
