package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Every number Pedro needs about THIS robot lives here.
 * Fill it in by running the AutoTune procedures in order
 * (open http://192.168.43.1:10158 while on the robot's Wi-Fi):
 *   1. Mecanum Procedure   -> paste its Java tab over drivetrainConfig
 *   2. Pinpoint Procedure  -> paste its Java tab over localizerConfig
 *   3. Foresight Procedure -> paste its Java tab over foresightConfig
 * Re-deploy after each paste, because the next tuner uses the previous result.
 *
 * JAVA NOTE: "c -> { ... }" is a lambda: a small function written inline.
 * Pedro calls it once with an empty config "c", and the lines inside fill it in.
 * You don't need to write lambdas yourself; AutoTune generates these blocks.
 */
public class Constants {

    // Starting values copied from YusufTele. The Mecanum tuner will confirm or fix the directions.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.0893265731691377);
        c.yPodOffset.set(-0.6979462480920506);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2366128331977277);
                Controller secondaryTranslationalForward = Controller.proportional(0.08742211945521583);
                Controller primaryTranslationalLateral = Controller.proportional(0.6546831501985335);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2418879305424033);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018412472697818934));
                c.brake.set(Controller.proportionalFeedforward(0.015650601793146094));

                c.headingFeedback.set(Controller.proportional(5.362870138294305));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04171353972750953, 0.010544798872343993));

                c.linearBrakeCoefficients.set(Matrix.diag(0.019771965347897658, 0.022242417546524174));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0027059726799062043, 0.002821049801153886));

                c.maxAchievableForwardVelocity.set(58.90819573119359);
                c.maxAchievableStrafeVelocity.set(46.22655225970419);
                c.naturalForwardDeceleration.set(77.4141316424814);
                c.naturalStrafeDeceleration.set(87.6223935354379);
            }
    );



    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
