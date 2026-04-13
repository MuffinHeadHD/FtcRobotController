package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.parts.IntakeMode

@TeleOp(name = "DanceParty", group = "Decode")
class DanceParty : LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        robot = Robot(this)


        waitForStart()

        while(opModeIsActive()) {
            robot.updateGamepadStates(false)
            robot.drive.drive(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble(), gamepad1.right_stick_x.toDouble())

              robot.turret.home()
                robot.turret.homeBiased(50)
                sleep(2000)
                robot.turret.homeBiased(-100)



                robot.spindexerLight.servo.setPosition(RobotConfig.lightServoPosition)


            robot.updateGamepadStates(true)
        }
    }
}