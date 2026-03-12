package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Crasher bot")
public class CrashBot extends OpMode {

    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor rf;
    private CRServo liftt;

    public void init() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        liftt = hardwareMap.get(CRServo.class, "liftt");

        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        double crab = gamepad1.left_stick_x;

        lf.setPower(drive + turn - crab);
        lb.setPower(drive + turn + crab);
        rb.setPower(drive - turn - crab);
        rf.setPower(drive - turn + crab);

        if (gamepad1.y) {
            liftt.setPower(1.0);
        } else if (gamepad1.a) {
            liftt.setPower(-1.0);
        } else{
            liftt.setPower(0.0);
        }
    }
}