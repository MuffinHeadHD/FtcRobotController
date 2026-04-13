package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

@Config
class BowlingBotConfig {
    companion object {
        @JvmField
        var speedFacto = 1.0
    }
}
@TeleOp(name = "Bowling Robot", group = "Decode")
public class BowlingRobot: LinearOpMode() {
    lateinit var lb: DcMotor
    lateinit var lf: DcMotor
    lateinit var rb: DcMotor
    lateinit var rf: DcMotor

    lateinit var ballDrop: Servo

    var dashboard: FtcDashboard = FtcDashboard.getInstance()


    override fun runOpMode() {
        lb = hardwareMap.get(DcMotor::class.java, "lb")
        lf = hardwareMap.get(DcMotor::class.java, "lf")
        rb = hardwareMap.get(DcMotor::class.java, "rb")
        rf = hardwareMap.get(DcMotor::class.java, "rf")

        lb.setDirection(DcMotorSimple.Direction.FORWARD)
        lf.setDirection(DcMotorSimple.Direction.REVERSE)
        rb.setDirection(DcMotorSimple.Direction.REVERSE)
        rf.setDirection(DcMotorSimple.Direction.REVERSE)

        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        ballDrop = hardwareMap.get(Servo::class.java, "ballDrop")

        waitForStart();

        while (opModeIsActive()) {
            var drive = -gamepad1.left_stick_y
            var turn = -gamepad1.right_stick_x
            var crab = -gamepad1.left_stick_x

            var lbPower = ((drive + turn - crab) * BowlingBotConfig.speedFacto)
            var lfPower = ((drive - turn - crab) * BowlingBotConfig.speedFacto)
            var rbPower = ((drive - turn + crab) * BowlingBotConfig.speedFacto)
            var rfPower = ((drive + turn + crab) * BowlingBotConfig.speedFacto)

            lb.setPower(lbPower)
            lf.setPower(lfPower)
            rb.setPower(rbPower)
            rf.setPower(rfPower)

            if (!gamepad1.a) {
                ballDrop.setPosition(0.55)
            } else {
                ballDrop.setPosition(0.8)
            }
        }

    }

}
