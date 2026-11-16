package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * The same auton for BLUE. The BIOBUZZ field is the same after turning it 180 degrees around its
 * center, so every red pose is rotated 180 degrees around (70.75, 70.75), the middle of the
 * field as the visualizer and Pedro model it (141.5 in; the manual says about 144 in). That moves each point to the opposite side AND turns its heading by 180.
 * (Do NOT use the visualizer's "Mirror" option for BIOBUZZ: mirroring is a different change.)
 *
 * JAVA NOTE: "extends BioBuzzVisualizerAuton" means this class gets ALL of that class's code.
 * "@Override" replaces just one method: makePoseFactory(). So when you change the red route,
 * blue changes too. No copying.
 */
@Autonomous(name = "BioBuzz Visualizer Auton (Blue)", group = "Competition", preselectTeleOp = "YusufTele")
public class BioBuzzVisualizerAutonBlue extends BioBuzzVisualizerAuton {

    @Override
    PoseFactory makePoseFactory() {
        return PoseFactory.degrees().rotateAround(70.75, 70.75, 180);
    }
}
