package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.parts.IntakeMode
import org.firstinspires.ftc.teamcode.parts.TurretConfig


@TeleOp(name = "Red All Gamepad 1", group = "Decode")
class RedAllControls : LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        robot = Robot(this)

        robot.turret.startLimelight()
        robot.turret.limelight.pipelineSwitch(1)

        waitForStart()

        robot.spindexer.home()

        while(opModeIsActive()) {
            robot.updateGamepadStates(false)
            robot.drive.drive(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble(), gamepad1.right_stick_x.toDouble())

            val intakeMode: IntakeMode = if (robot.gamepadState1.b) IntakeMode.IN else if (robot.gamepadState1.y) IntakeMode.OUT else IntakeMode.OFF
            robot.intake.set(intakeMode)

            if (robot.gamepadState1.left_bumper && !robot.lastGamepadState1.left_bumper) {
                robot.spindexer.rotate(-1)
            }
            if (robot.gamepadState1.right_bumper && !robot.lastGamepadState1.right_bumper) {
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

            if (robot.gamepadState2.dpad_up) {
                TurretConfig.FlywheelLowVoltageAdditive = 0.05
            } else if (robot.gamepadState2.dpad_down) {
                TurretConfig.FlywheelLowVoltageAdditive = 0.1
            } else if (robot.gamepadState2.dpad_left) {
                TurretConfig.FlywheelLowVoltageAdditive = 0.2
            } else if (robot.gamepadState2.dpad_right) {
                TurretConfig.FlywheelLowVoltageAdditive = 0.0
            }


            robot.dashboardTelemetry.addData("Lime Dist", robot.turret.limelight.latestResult.tx)
            robot.dashboardTelemetry.addData("Turret Power", robot.turret.turretMotor.power)
            robot.dashboardTelemetry.addData("txToUse", robot.turret.turretMotor.targetPosition)
            robot.dashboardTelemetry.addData("txToUse", robot.turret.turretMotor.currentPosition)


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