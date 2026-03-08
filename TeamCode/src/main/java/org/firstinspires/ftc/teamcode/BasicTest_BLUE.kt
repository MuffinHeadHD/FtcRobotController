package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.parts.IntakeMode

@TeleOp(name = "GOOD_BLUE")
class BasicTest_BLUE : LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        robot = Robot(this)

        robot.turret.startLimelight()
        robot.turret.limelight.pipelineSwitch(0)

        waitForStart()

        robot.spindexer.home()

        while(opModeIsActive()) {
            robot.updateGamepadStates(false)
            robot.drive.drive(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble(), gamepad1.right_stick_x.toDouble())

            val intakeMode: IntakeMode = if (robot.gamepadState2.b) IntakeMode.IN else if (robot.gamepadState2.y) IntakeMode.OUT else IntakeMode.OFF
            robot.intake.set(intakeMode)

            if (robot.gamepadState2.left_bumper && !robot.lastGamepadState2.left_bumper) {
                robot.spindexer.rotate(-1)
            }
            if (robot.gamepadState2.right_bumper && !robot.lastGamepadState2.right_bumper) {
                robot.spindexer.rotate(1)
            }

            if (robot.gamepadState1.y && !robot.lastGamepadState1.y) {
                robot.turret.home()
            }

            if (robot.gamepadState1.dpad_up) {
                robot.lift.up()
            } else if (robot.gamepadState1.dpad_down) {
                robot.lift.down()
            } else {
                robot.lift.stop()
            }


            robot.dashboardTelemetry.addData("Lime Dist", robot.turret.limelight.latestResult.tx)
            robot.dashboardTelemetry.addData("Turret Power", robot.turret.turretMotor.power)

            val hsv = FloatArray(3)
            Color.RGBToHSV(robot.spindexer.colorSensor.red(), robot.spindexer.colorSensor.green(), robot.spindexer.colorSensor.blue(), hsv)
            robot.dashboardTelemetry.addData("H", hsv[0])
            robot.dashboardTelemetry.addData("S", hsv[1])
            robot.dashboardTelemetry.addData("V", hsv[2])
            robot.dashboardTelemetry.update()

            robot.update()

            robot.updateGamepadStates(true)
        }
    }
}