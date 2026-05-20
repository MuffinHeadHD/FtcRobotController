package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.parts.IntakeMode
import org.firstinspires.ftc.teamcode.parts.Turret
import org.firstinspires.ftc.teamcode.parts.TurretConfig

@TeleOp(name = "Red Teleop", group = "Decode")
class BasicTest_RED : LinearOpMode() {
    lateinit var robot: Robot


    override fun runOpMode() {
        robot = Robot(this)

        robot.turret.startLimelight()
        robot.turret.limelight.pipelineSwitch(1)

        waitForStart()

        robot.spindexer.home()

        var toggleBack = false
        var lastBack = false

        while(opModeIsActive()) {

            var backPressed = robot.gamepadState1.back

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

            if (backPressed && !lastBack) {
                toggleBack = !toggleBack
            }

            lastBack = backPressed

            if (toggleBack) {
                robot.turret.shooting = true
            } else {
                robot.turret.shooting = false
            }

            if (robot.gamepadState2.dpad_right) {
                TurretConfig.FlywheelLowVoltageAdditive = TurretConfig.FlywheelLowVoltageAdditive+ 0.005
            } else if (robot.gamepadState2.dpad_left) {
                TurretConfig.FlywheelLowVoltageAdditive = TurretConfig.FlywheelLowVoltageAdditive - 0.005
            } else if (robot.gamepadState2.x && robot.gamepadState2.a && robot.gamepadState2.b && robot.gamepadState2.y) {
                TurretConfig.FlywheelLowVoltageAdditive = 0.0
            }


            robot.dashboardTelemetry.addData("Lime Dist", robot.turret.limelight.latestResult.tx)
            robot.dashboardTelemetry.addData("Turret Power", robot.turret.turretMotor.power)
            robot.dashboardTelemetry.addData("txToUse", robot.turret.turretMotor.targetPosition)
            robot.dashboardTelemetry.addData("txToUse", robot.turret.turretMotor.currentPosition)
            robot.dashboardTelemetry.addData("FlyWheel power added", TurretConfig.FlywheelLowVoltageAdditive)
            robot.dashboardTelemetry.addData("ball in the left", robot.spindexer.ballPresentLeft)
            robot.dashboardTelemetry.addData("ball in the right", robot.spindexer.ballPresentRight)
            robot.dashboardTelemetry.addData("ball in the front", robot.spindexer.ballPresentFront)
// hi

            val hsvRight = FloatArray(3)
            val hsvLeft = FloatArray(3)
            val hsvFront = FloatArray(3)
            Color.RGBToHSV(robot.spindexer.colorSensorRight.red(), robot.spindexer.colorSensorRight.green(), robot.spindexer.colorSensorRight.blue(), hsvRight)
            Color.RGBToHSV(robot.spindexer.colorSensorLeft.red(), robot.spindexer.colorSensorLeft.green(), robot.spindexer.colorSensorLeft.blue(), hsvLeft)
            Color.RGBToHSV(robot.spindexer.colorSensorFront.red(), robot.spindexer.colorSensorFront.green(), robot.spindexer.colorSensorFront.blue(), hsvFront)
            robot.dashboardTelemetry.addData("H", hsvRight[0])
            robot.dashboardTelemetry.addData("S", hsvRight[1])
            robot.dashboardTelemetry.addData("V", hsvRight[2])
            robot.dashboardTelemetry.update()

            robot.update()

            robot.updateGamepadStates(true)
        }
    }
}