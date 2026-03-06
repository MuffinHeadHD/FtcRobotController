package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Crasher bot")
public class CrashBot extends OpMode {

    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor rf;

    public void init() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double crab = gamepad1.left_stick_x;

        lf.setPower(drive + turn - crab);
        lb.setPower(drive + turn + crab);
        rb.setPower(drive - turn - crab);
        rf.setPower(drive - turn + crab);

    }
}