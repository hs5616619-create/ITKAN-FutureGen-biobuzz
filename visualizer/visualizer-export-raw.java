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

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

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
