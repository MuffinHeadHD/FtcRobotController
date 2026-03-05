package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot

@TeleOp(name = "SpindexerTest")
class SpindexerTest: LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        robot = Robot(this)

        waitForStart()

        robot.spindexer.home()

        while (opModeIsActive()) {
            robot.updateGamepadStates(false)

            if (robot.gamepadState1.a && !robot.lastGamepadState1.a) {
                robot.spindexer.rotate(1)
            } else if (robot.gamepadState1.b && !robot.lastGamepadState1.b) {
                robot.spindexer.rotate(-1)
            }

            robot.update()

            robot.dashboardTelemetry.addData("Current", robot.spindexer.servo.position)
            robot.dashboardTelemetry.addData("Target", robot.spindexer.servo.targetPosition)
            robot.dashboardTelemetry.addData("Error", robot.spindexer.servo.targetPosition - robot.spindexer.servo.position)
            robot.dashboardTelemetry.addData("Power", robot.spindexer.servo.servo.power)
            robot.dashboardTelemetry.update()

            robot.updateGamepadStates(true)
        }
    }
}