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
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.pedropathing.ivy.groups.Groups.race;

@Autonomous(name = "HamzaPedro", group = "Autonomous")
public class HamzaPedro extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.0957, 7.5436, 90);
    private final Pose path1 = poseFactory.of(59, 9, 90);
    private final Pose penStrokeStart = poseFactory.of(59, 9, 90);
    private final Pose penStroke = poseFactory.of(59, 34.6268, 90);
    private final Pose penStrokeThrough1 = poseFactory.of(59, 9, 0);
    private final Pose penStrokeThrough2 = poseFactory.of(59, 14, 0);
    private final Pose penStrokeThrough3 = poseFactory.of(59, 17, 0);
    private final Pose penStrokeThrough4 = poseFactory.of(59, 20, 0);
    private final Pose penStrokeThrough5 = poseFactory.of(59, 24, 0);
    private final Pose penStrokeThrough6 = poseFactory.of(59, 27, 0);
    private final Pose penStrokeThrough7 = poseFactory.of(59, 31, 0);
    private final Pose penStroke_2Start = poseFactory.of(59, 34.6268, 90);
    private final Pose penStroke_2 = poseFactory.of(15, 35, 90);
    private final Pose penStroke_2Through1 = poseFactory.of(54, 35, 0);
    private final Pose penStroke_2Through2 = poseFactory.of(48, 35, 0);
    private final Pose penStroke_2Through3 = poseFactory.of(42, 35, 0);
    private final Pose penStroke_2Through4 = poseFactory.of(37, 35, 0);
    private final Pose penStroke_2Through5 = poseFactory.of(32, 35, 0);
    private final Pose penStroke_2Through6 = poseFactory.of(26, 35, 0);
    private final Pose penStroke_2Through7 = poseFactory.of(21, 35, 0);
    private final Pose penStroke_3Start = poseFactory.of(15, 35, 90);
    private final Pose penStroke_3 = poseFactory.of(15, 46, 1);
    private final Pose penStroke_3Through1 = poseFactory.of(15, 36.5, 0);
    private final Pose penStroke_3Through2 = poseFactory.of(15, 37, 0);
    private final Pose penStroke_3Through3 = poseFactory.of(15, 38.5, 0);
    private final Pose penStroke_3Through4 = poseFactory.of(15, 40, 0);
    private final Pose penStroke_3Through5 = poseFactory.of(15, 41.5, 0);
    private final Pose penStroke_3Through6 = poseFactory.of(15, 43, 0);
    private final Pose penStroke_3Through7 = poseFactory.of(15, 44.5, 0);
    private final Pose penStroke_4Start = poseFactory.of(15, 46, 0);
    private final Pose penStroke_4 = poseFactory.of(10, 46.5, 0);
    private final Pose penStroke_4Through1 = poseFactory.of(15, 46.5, 0);
    private final Pose penStroke_4Through2 = poseFactory.of(14, 46.5, 0);
    private final Pose penStroke_4Through3 = poseFactory.of(13, 46.5, 0);
    private final Pose penStroke_4Through4 = poseFactory.of(12, 46.5, 0);
    private final Pose penStroke_4Through5 = poseFactory.of(11, 46.5, 0);
    private final Pose penStroke_4Through6 = poseFactory.of(10, 46.5, 0);
    private final Pose penStroke_5Start = poseFactory.of(10, 46.5, 0);
    private final Pose penStroke_5 = poseFactory.of(32.8627, 46.5, 90);
    private final Pose penStroke_5Through1 = poseFactory.of(12, 46.5, 0);
    private final Pose penStroke_5Through2 = poseFactory.of(15.1876, 46.5, 0);
    private final Pose penStroke_5Through3 = poseFactory.of(18, 46.5, 0);
    private final Pose penStroke_5Through4 = poseFactory.of(21, 46.5, 0);
    private final Pose penStroke_5Through5 = poseFactory.of(24, 46.5, 0);
    private final Pose penStroke_5Through6 = poseFactory.of(27, 46.5, 0);
    private final Pose penStroke_5Through7 = poseFactory.of(30, 46.5, 0);
    private final Pose penStroke_6Start = poseFactory.of(32.8627, 46.5, 90);
    private final Pose penStroke_6 = poseFactory.of(33, 82, 90);
    private final Pose penStroke_6Through1 = poseFactory.of(33, 47, 0);
    private final Pose penStroke_6Through2 = poseFactory.of(33, 52, 0);
    private final Pose penStroke_6Through3 = poseFactory.of(33, 57, 0);
    private final Pose penStroke_6Through4 = poseFactory.of(33, 62, 0);
    private final Pose penStroke_6Through5 = poseFactory.of(33, 67, 0);
    private final Pose penStroke_6Through6 = poseFactory.of(33, 72, 0);
    private final Pose penStroke_6Through7 = poseFactory.of(33, 77, 0);
    private final Pose penStroke_7Start = poseFactory.of(33, 82, 90);
    private final Pose penStroke_7 = poseFactory.of(33, 117.9042, 90);
    private final Pose penStroke_7Through1 = poseFactory.of(33, 85.0768, 0);
    private final Pose penStroke_7Through2 = poseFactory.of(33, 89.8895, 0);
    private final Pose penStroke_7Through3 = poseFactory.of(33, 94.7058, 0);
    private final Pose penStroke_7Through4 = poseFactory.of(33, 99.5201, 0);
    private final Pose penStroke_7Through5 = poseFactory.of(33, 104.3365, 0);
    private final Pose penStroke_7Through6 = poseFactory.of(33, 109.1529, 0);
    private final Pose penStroke_7Through7 = poseFactory.of(33, 113.9691, 0);
    private final Pose penStroke_8Start = poseFactory.of(33, 117.9042, 360);
    private final Pose penStroke_8 = poseFactory.of(59.5278, 118, 270);
    private final Pose penStroke_8Through1 = poseFactory.of(39, 118, 0);
    private final Pose penStroke_8Through2 = poseFactory.of(42.6042, 118, 0);
    private final Pose penStroke_8Through3 = poseFactory.of(45.9945, 118, 0);
    private final Pose penStroke_8Through4 = poseFactory.of(49.3922, 118, 0);
    private final Pose penStroke_8Through5 = poseFactory.of(52.7705, 118, 0);
    private final Pose penStroke_8Through6 = poseFactory.of(56.1531, 118, 0);
    private final Pose penStroke_9Start = poseFactory.of(59.5278, 118, 270);
    private final Pose penStroke_9 = poseFactory.of(59, 126.978, 270);
    private final Pose penStroke_9Through1 = poseFactory.of(59, 118.4388, 0);
    private final Pose penStroke_9Through2 = poseFactory.of(59, 119.6601, 0);
    private final Pose penStroke_9Through3 = poseFactory.of(59, 120.8815, 0);
    private final Pose penStroke_9Through4 = poseFactory.of(59, 122.1007, 0);
    private final Pose penStroke_9Through5 = poseFactory.of(59, 123.321, 0);
    private final Pose penStroke_9Through6 = poseFactory.of(59, 124.539, 0);
    private final Pose penStroke_9Through7 = poseFactory.of(59, 125.7601, 0);
    private final Pose penStroke_10Start = poseFactory.of(59, 126.978, 270);
    private final Pose penStroke_10 = poseFactory.of(47.1506, 127, 270);
    private final Pose penStroke_10Through1 = poseFactory.of(58.1666, 127, 0);
    private final Pose penStroke_10Through2 = poseFactory.of(56.5889, 127, 0);
    private final Pose penStroke_10Through3 = poseFactory.of(55.0067, 127, 0);
    private final Pose penStroke_10Through4 = poseFactory.of(53.4248, 127, 0);
    private final Pose penStroke_10Through5 = poseFactory.of(51.8502, 127, 0);
    private final Pose penStroke_10Through6 = poseFactory.of(50.3002, 127, 0);
    private final Pose penStroke_10Through7 = poseFactory.of(48.7328, 127, 0);
    private final Pose penStroke_11Start = poseFactory.of(47.1506, 127, 270);
    private final Pose penStroke_11 = poseFactory.of(46.9048, 132.8827, 270);
    private final Pose penStroke_11Through1 = poseFactory.of(47.0392, 129.7453, 0);
    private final Pose penStroke_12Start = poseFactory.of(46.9048, 132.8827, 270);
    private final Pose penStroke_12 = poseFactory.of(59.0017, 133, 270);
    private final Pose penStroke_12Through1 = poseFactory.of(48.2187, 133, 0);
    private final Pose penStroke_12Through2 = poseFactory.of(49.7451, 133, 0);
    private final Pose penStroke_12Through3 = poseFactory.of(51.2715, 133, 0);
    private final Pose penStroke_12Through4 = poseFactory.of(52.7979, 133, 0);
    private final Pose penStroke_12Through5 = poseFactory.of(54.3243, 133, 0);
    private final Pose penStroke_12Through6 = poseFactory.of(55.8367, 133, 0);
    private final Pose penStroke_12Through7 = poseFactory.of(57.3576, 133, 0);
    private final Pose penStroke_13 = poseFactory.of(59, 118, -90.0065);
    private final Pose penStroke_13Through1 = poseFactory.of(59, 130, 0);
    private final Pose penStroke_13Through2 = poseFactory.of(59, 127, 0);
    private final Pose penStroke_13Through3 = poseFactory.of(59, 125, 0);
    private final Pose penStroke_13Through4 = poseFactory.of(59, 122, 0);
    private final Pose penStroke_14 = poseFactory.of(59, 107.3347, -90.7511);
    private final Pose penStroke_14Through1 = poseFactory.of(59, 116.7077, 0);
    private final Pose penStroke_14Through2 = poseFactory.of(59, 115.3633, 0);
    private final Pose penStroke_14Through3 = poseFactory.of(59.0015, 114.0302, 0);
    private final Pose penStroke_14Through4 = poseFactory.of(59, 112.6971, 0);
    private final Pose penStroke_14Through5 = poseFactory.of(59, 111.359, 0);
    private final Pose penStroke_14Through6 = poseFactory.of(59, 110.0256, 0);
    private final Pose penStroke_14Through7 = poseFactory.of(59, 108.6822, 0);
    private final Pose penStroke_15Start = poseFactory.of(59, 107.3347, 270);
    private final Pose penStroke_15 = poseFactory.of(8, 107, 270);
    private final Pose penStroke_15Through1 = poseFactory.of(52.401, 107, 0);
    private final Pose penStroke_15Through2 = poseFactory.of(45.4868, 107, 0);
    private final Pose penStroke_15Through3 = poseFactory.of(38.5725, 107, 0);
    private final Pose penStroke_15Through4 = poseFactory.of(31.6582, 107, 0);
    private final Pose penStroke_15Through5 = poseFactory.of(24.7538, 107, 0);
    private final Pose penStroke_15Through6 = poseFactory.of(17.8418, 107, 0);
    private final Pose penStroke_15Through7 = poseFactory.of(10.9279, 107, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, penStroke()),
                follow(follower, penStroke_2()),
                follow(follower, penStroke_3()),
                follow(follower, penStroke_4()),
                follow(follower, penStroke_5()),
                follow(follower, path7()),
                follow(follower, penStroke_6()),
                follow(follower, penStroke_7()),
                follow(follower, penStroke_8()),
                follow(follower, penStroke_9()),
                follow(follower, penStroke_10()),
                follow(follower, penStroke_11()),
                follow(follower, penStroke_12()),
                follow(follower, penStroke_13())
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

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path penStroke() {
        return Paths.through(penStrokeStart, penStrokeThrough1, penStrokeThrough2, penStrokeThrough3, penStrokeThrough4, penStrokeThrough5, penStrokeThrough6, penStrokeThrough7, penStroke).linear(penStrokeStart, penStroke);
    }

    public Path penStroke_2() {
        return Paths.through(penStroke_2Start, penStroke_2Through1, penStroke_2Through2, penStroke_2Through3, penStroke_2Through4, penStroke_2Through5, penStroke_2Through6, penStroke_2Through7, penStroke_2).linear(penStroke_2Start, penStroke_2);
    }

    public Path penStroke_3() {
        return Paths.through(penStroke_3Start, penStroke_3Through1, penStroke_3Through2, penStroke_3Through3, penStroke_3Through4, penStroke_3Through5, penStroke_3Through6, penStroke_3Through7, penStroke_3).linear(penStroke_3Start, penStroke_3);
    }

    public Path penStroke_4() {
        return Paths.through(penStroke_4Start, penStroke_4Through1, penStroke_4Through2, penStroke_4Through3, penStroke_4Through4, penStroke_4Through5, penStroke_4Through6, penStroke_4).linear(penStroke_4Start, penStroke_4);
    }

    public Path penStroke_5() {
        return Paths.through(penStroke_5Start, penStroke_5Through1, penStroke_5Through2, penStroke_5Through3, penStroke_5Through4, penStroke_5Through5, penStroke_5Through6, penStroke_5Through7, penStroke_5).linear(penStroke_5Start, penStroke_5);
    }

    public Path path7() {
        return Paths.path(Paths.through(penStroke_6Start, penStroke_6Through1, penStroke_6Through2, penStroke_6Through3, penStroke_6Through4, penStroke_6Through5, penStroke_6Through6, penStroke_6Through7, penStroke_6).linear(penStroke_6Start, penStroke_6), Paths.through(penStroke_7Start, penStroke_7Through1, penStroke_7Through2, penStroke_7Through3, penStroke_7Through4, penStroke_7Through5, penStroke_7Through6, penStroke_7Through7, penStroke_7).linear(penStroke_7Start, penStroke_7));
    }

    public Path penStroke_6() {
        return Paths.through(penStroke_8Start, penStroke_8Through1, penStroke_8Through2, penStroke_8Through3, penStroke_8Through4, penStroke_8Through5, penStroke_8Through6, penStroke_8).linear(penStroke_8Start, penStroke_8);
    }

    public Path penStroke_7() {
        return Paths.through(penStroke_9Start, penStroke_9Through1, penStroke_9Through2, penStroke_9Through3, penStroke_9Through4, penStroke_9Through5, penStroke_9Through6, penStroke_9Through7, penStroke_9).linear(penStroke_9Start, penStroke_9);
    }

    public Path penStroke_8() {
        return Paths.through(penStroke_10Start, penStroke_10Through1, penStroke_10Through2, penStroke_10Through3, penStroke_10Through4, penStroke_10Through5, penStroke_10Through6, penStroke_10Through7, penStroke_10).linear(penStroke_10Start, penStroke_10);
    }

    public Path penStroke_9() {
        return Paths.through(penStroke_11Start, penStroke_11Through1, penStroke_11).linear(penStroke_11Start, penStroke_11);
    }

    public Path penStroke_10() {
        return Paths.through(penStroke_12Start, penStroke_12Through1, penStroke_12Through2, penStroke_12Through3, penStroke_12Through4, penStroke_12Through5, penStroke_12Through6, penStroke_12Through7, penStroke_12).linear(penStroke_12Start, penStroke_12);
    }

    public Path penStroke_11() {
        return Paths.through(penStroke_12, penStroke_13Through1, penStroke_13Through2, penStroke_13Through3, penStroke_13Through4, penStroke_13).tangent();
    }

    public Path penStroke_12() {
        return Paths.through(penStroke_13, penStroke_14Through1, penStroke_14Through2, penStroke_14Through3, penStroke_14Through4, penStroke_14Through5, penStroke_14Through6, penStroke_14Through7, penStroke_14).tangent();
    }

    public Path penStroke_13() {
        return Paths.through(penStroke_15Start, penStroke_15Through1, penStroke_15Through2, penStroke_15Through3, penStroke_15Through4, penStroke_15Through5, penStroke_15Through6, penStroke_15Through7, penStroke_15).linear(penStroke_15Start, penStroke_15);
    }
}
