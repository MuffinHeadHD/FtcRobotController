package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DigitalChannel.*
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.parts.AxonServo
import org.firstinspires.ftc.teamcode.parts.Drive
import org.firstinspires.ftc.teamcode.parts.Intake
import org.firstinspires.ftc.teamcode.parts.Lift
import org.firstinspires.ftc.teamcode.parts.Light
import org.firstinspires.ftc.teamcode.parts.Spindexer
import org.firstinspires.ftc.teamcode.parts.Turret
import org.firstinspires.ftc.teamcode.parts.Updatable
import org.firstinspires.ftc.teamcode.util.GamepadState

@Config
class RobotConfig {

    companion object{

        @JvmField
        var lightServoPosition = 0.52
    }
}

open class Robot(val opMode: OpMode) {
    lateinit var lf: DcMotor
    lateinit var lb: DcMotor
    lateinit var rf: DcMotor
    lateinit var rb: DcMotor
    lateinit var driveMotors: Array<DcMotor>

    lateinit var flywheelMotor: DcMotorEx
    lateinit var intakeServo: CRServo
    lateinit var spindexerServo: AxonServo
    lateinit var spindexerColorRight: ColorSensor
    lateinit var spindexerColorLeft: ColorSensor
    lateinit var spindexerColorFront: ColorSensor
    lateinit var spindexerDistance: DistanceSensor
    lateinit var spindexerMagnet: TouchSensor
    lateinit var launchDist: AnalogInput
    lateinit var backlightR: Servo
    lateinit var backlightL: Servo
    lateinit var voltageSensor: VoltageSensor


    lateinit var turretMotor: DcMotor
    lateinit var limelight: Limelight3A

    lateinit var leftLiftServo: CRServo
    lateinit var rightLiftServo: CRServo

    lateinit var spindexerLight: Light

    lateinit var drive: Drive
    lateinit var intake: Intake
    lateinit var turret: Turret
    lateinit var spindexer: Spindexer
    lateinit var lift: Lift


    lateinit var updateables: Array<Updatable>

    var dashboard: FtcDashboard = FtcDashboard.getInstance()
    var dashboardTelemetry: Telemetry = dashboard.telemetry

    lateinit var gamepadState1: GamepadState
    lateinit var lastGamepadState1: GamepadState
    lateinit var gamepadState2: GamepadState
    lateinit var lastGamepadState2: GamepadState

    init {
        val hwMap: HardwareMap = opMode.hardwareMap

        launchDist = hwMap.get(AnalogInput::class.java, "launchDist")
        limelight = hwMap.get(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
        limelight.start()

        voltageSensor = opMode.hardwareMap.voltageSensor.iterator().next()


        lf = hwMap.get(DcMotor::class.java, "lf")
        lb = hwMap.get(DcMotor::class.java, "lb")
        rf = hwMap.get(DcMotor::class.java, "rf")
        rb = hwMap.get(DcMotor::class.java, "rb")

        backlightL = hwMap.get(Servo::class.java, "backlightL")
        backlightR = hwMap.get(Servo::class.java, "backlightR")
        driveMotors = arrayOf(lf, lb, rf, rb)

        for (motor in driveMotors) {
            motor.power = 0.0
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        lf.direction = Direction.REVERSE
        lb.direction = Direction.REVERSE
        rf.direction = Direction.REVERSE
        rb.direction = Direction.FORWARD

        flywheelMotor = hwMap.get(DcMotorEx::class.java, "flywheel")
        flywheelMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheelMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        intakeServo = hwMap.get(CRServo::class.java, "intk")

        spindexerServo = AxonServo(
            hwMap.get(CRServo::class.java, "spdx"),
            hwMap.get(AnalogInput::class.java, "axen")
        )
        spindexerColorRight = hwMap.get(ColorSensor::class.java, "csRight")
        spindexerColorFront = hwMap.get(ColorSensor::class.java, "csFront")
        spindexerColorLeft = hwMap.get(ColorSensor::class.java, "csLeft")
        spindexerDistance = hwMap.get(DistanceSensor::class.java, "csRight")
        spindexerMagnet = hwMap.get(TouchSensor::class.java, "mag")

        turretMotor = hwMap.get(DcMotor::class.java, "tt")
        turretMotor.direction = Direction.REVERSE
        turretMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turretMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

//        leftLiftServo = hwMap.get(CRServo::class.java, "leftLift")
//        rightLiftServo = hwMap.get(CRServo::class.java, "rightLift")
//        leftLiftServo.direction = Direction.FORWARD
//        rightLiftServo.direction = Direction.FORWARD

        spindexerLight = Light(hwMap.get(Servo::class.java, "prism"))

        drive = Drive(lf, lb, rf, rb)
        intake = Intake(intakeServo)
        spindexer = Spindexer(spindexerServo,spindexerDistance,spindexerColorFront,spindexerColorLeft,spindexerColorRight, spindexerLight,spindexerMagnet,false,false, false, false)
        turret = Turret(limelight, turretMotor, flywheelMotor, launchDist, backlightR, backlightL, spindexer, shooting = true)

        lift = Lift(hwMap.get(CRServo::class.java, "leftlift"), hwMap.get(CRServo::class.java, "rightlift"))


        updateables = arrayOf(spindexer, turret)

        gamepadState1 = GamepadState()
        lastGamepadState1 = GamepadState()
        gamepadState2 = GamepadState()
        lastGamepadState2 = GamepadState()
    }
        var lastVoltage = 0.0
        var voltageDropPerMin = 0.0
        var lastVoltageTime = System.nanoTime()

    fun update() {
        updateables.forEach { it.update() }

        val now = System.nanoTime()
        val dt = (now - lastVoltageTime) / 1e9
        lastVoltageTime = now

        val voltage = voltageSensor.voltage

        if (lastVoltage != 0.0 && dt > 0.0) {
            voltageDropPerMin = (((lastVoltage - voltage) / dt) * 60.0)
        }

        lastVoltage = voltage

        if (spindexer.SpindexerShootingReady == true) {
            turret.shooting = true
        }

        dashboardTelemetry.addData("voltage", voltage)
        dashboardTelemetry.addData("Battery drain (volts per min)", voltageDropPerMin)
        dashboardTelemetry.addData("Flywheel Velocity", flywheelMotor.velocity)
        dashboardTelemetry.addData("Flywheel Goal", turret.flyPower)
        dashboardTelemetry.update()
    }


    fun updateGamepadStates(last: Boolean) {
        if (last) {
            lastGamepadState1.updateGamepadState(gamepadState1)
            lastGamepadState2.updateGamepadState(gamepadState2)
        } else {
            gamepadState1.updateGamepadState(opMode.gamepad1)
            gamepadState2.updateGamepadState(opMode.gamepad2)
        }
    }
}