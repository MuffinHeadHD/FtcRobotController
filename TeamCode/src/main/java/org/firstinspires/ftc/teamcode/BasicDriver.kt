package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.parts.IntakeMode

@TeleOp(name = "Driving and Lift", group = "Decode")
class BasicDriver : LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        robot = Robot(this)

        waitForStart()

        while(opModeIsActive()) {
            robot.updateGamepadStates(false)
            robot.drive.drive(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble(), gamepad1.right_stick_x.toDouble())
            robot.flywheelMotor.power = 0.0

            val intakeMode: IntakeMode = if (robot.gamepadState1.b) IntakeMode.IN else if (robot.gamepadState1.y) IntakeMode.OUT else IntakeMode.OFF
            robot.intake.set(intakeMode)

            if (robot.gamepadState1.dpad_up) {
                robot.lift.up()
            } else if (robot.gamepadState1.dpad_down) {
                robot.lift.down()
            } else {
                robot.lift.stop()
            }

            robot.updateGamepadStates(true)
        }
    }
}