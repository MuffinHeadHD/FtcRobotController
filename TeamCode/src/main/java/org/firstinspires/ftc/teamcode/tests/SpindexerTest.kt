package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot

@TeleOp(name = "SpindexerTest")
class SpindexerTest: LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        robot = Robot(this)

        robot.spindexer.home()

        waitForStart()

        while (opModeIsActive()) {
            robot.update()

            if (gamepad1.left_bumper && !robot.gamepadState1.left_bumper) {
                robot.spindexer.rotate(1)
            }

            robot.updateGamepadStates()
        }
    }
}