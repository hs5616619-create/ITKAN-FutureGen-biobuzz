package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// One-time check: shows every motor's encoder count. Nothing moves.
// Turn one wheel (or the flywheel) by hand; the name whose number changes is
// the port that wheel's encoder cable is plugged into. A number that never
// changes is an unplugged encoder. Write the results in MainTeleOp's header.
@TeleOp(name = "Encoder Check", group = "Test")
public class EncoderCheck extends OpMode {

    String[] names = {"fl", "fr", "bl", "br", "sl", "sr", "tl", "tr"};
    DcMotor[] motors = new DcMotor[8];

    @Override
    public void init() {
        for (int i = 0; i < 8; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, names[i]);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void init_loop() {
        for (int i = 0; i < 8; i++) {
            telemetry.addData(names[i], motors[i].getCurrentPosition());
        }
    }

    @Override
    public void loop() {
        init_loop();
    }
}
