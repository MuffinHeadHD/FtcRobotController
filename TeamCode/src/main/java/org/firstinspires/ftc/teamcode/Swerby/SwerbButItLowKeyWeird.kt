package org.firstinspires.ftc.teamcode.Swerby

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion

@TeleOp

@Config
class SwerbButItLowKeyWeird {

    @TeleOp(name = "Swerb I think")
    public class Swerby : LinearOpMode() {
        lateinit var one: DcMotor
        lateinit var two: DcMotor
        lateinit var three: DcMotor
        lateinit var four: DcMotor

        lateinit var encoderLeft : AnalogInput
        lateinit var encoderRight: AnalogInput


        override fun runOpMode() {
            val hwMap: HardwareMap = BlocksOpModeCompanion.opMode.hardwareMap
            one = hwMap.get(DcMotor::class.java, "one")
            two = hwMap.get(DcMotor::class.java, "two")
            three = hwMap.get(DcMotor::class.java, "three")
            four = hwMap.get(DcMotor::class.java, "four")

            encoderLeft = hwMap.get(AnalogInput::class.java, "encoderLeft")
            encoderRight = hwMap.get(AnalogInput::class.java, "encoderRight")

            one.setDirection(DcMotorSimple.Direction.FORWARD)
            two.setDirection(DcMotorSimple.Direction.REVERSE)
            three.setDirection(DcMotorSimple.Direction.REVERSE)
            four.setDirection(DcMotorSimple.Direction.REVERSE)

            one.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            two.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            three.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            four.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

            one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            four.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)


            waitForStart();

            while (opModeIsActive()) {
                var leftPodVolts = encoderLeft.getVoltage()
                var leftPodAngle = (leftPodVolts * 0.53571)

                var rightPodVolts = encoderRight.getVoltage()
                var rightPodAngle = ( rightPodVolts * 0.53571)


                var drive = -gamepad1.left_stick_y
                var turn = -gamepad1.right_stick_x

                var OnePower = (drive + turn) * 1.0
                var TwoPower = (drive - turn) * 1.0
                var ThreePower = (drive - turn) * 1.0
                var FourPower = (drive + turn) * 1.0

                one.setPower(OnePower)
                two.setPower(TwoPower)
                three.setPower(ThreePower)
                four.setPower(FourPower)
            }

        }

    }
}