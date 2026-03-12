package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "Move Frontt")
public class CrasherBoto extends LinearOpMode {

    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor rf;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
    lf = hardwareMap.get(DcMotor.class, "lf");
    lb = hardwareMap.get(DcMotor.class, "lb");
    rf = hardwareMap.get(DcMotor.class, "rf");
    rb = hardwareMap.get(DcMotor.class, "rb");

    lf.setDirection(DcMotor.Direction.FORWARD);
    lb.setDirection(DcMotor.Direction.FORWARD);
    rf.setDirection(DcMotor.Direction.REVERSE);
    rb.setDirection(DcMotor.Direction.REVERSE);

    lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    telemetry.addData("left front", lf.getPower());
    telemetry.addData("left back", lb.getPower());
    telemetry.addData("right front", rf.getPower());
    telemetry.addData("right back", rb.getPower());

        waitForStart();

        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 1500.0) {
            lf.setPower(-0.6);
            lb.setPower(-0.6);
            rf.setPower(-0.6);
            rb.setPower(-0.6);

            telemetry.update();
        }
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        telemetry.update();

        sleep(500);


    }
}
