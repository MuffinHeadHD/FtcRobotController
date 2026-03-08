package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
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

open class Robot(val opMode: OpMode) {
    lateinit var lf: DcMotor
    lateinit var lb: DcMotor
    lateinit var rf: DcMotor
    lateinit var rb: DcMotor
    lateinit var driveMotors: Array<DcMotor>

    lateinit var flywheelMotor: DcMotorEx
    lateinit var intakeServo: CRServo
    lateinit var spindexerServo: AxonServo
    lateinit var spindexerColor: ColorSensor
    lateinit var spindexerDistance: DistanceSensor
    lateinit var spindexerMagnet: TouchSensor

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
        val hardwareMap: HardwareMap = opMode.hardwareMap

        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
        limelight.start()

        lf = hardwareMap.get(DcMotor::class.java, "lf")
        lb = hardwareMap.get(DcMotor::class.java, "lb")
        rf = hardwareMap.get(DcMotor::class.java, "rf")
        rb = hardwareMap.get(DcMotor::class.java, "rb")
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

        flywheelMotor = hardwareMap.get(DcMotorEx::class.java, "flywheel")
        flywheelMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheelMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        intakeServo = hardwareMap.get(CRServo::class.java, "intk")

        spindexerServo = AxonServo(
            hardwareMap.get(CRServo::class.java, "spdx"),
            hardwareMap.get(AnalogInput::class.java, "axen")
        )
        spindexerColor = hardwareMap.get(ColorSensor::class.java, "css")
        spindexerDistance = hardwareMap.get(DistanceSensor::class.java, "cs")
        spindexerMagnet = hardwareMap.get(TouchSensor::class.java, "mag")

        turretMotor = hardwareMap.get(DcMotor::class.java, "tt")
        turretMotor.direction = Direction.REVERSE
        turretMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turretMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        leftLiftServo = hardwareMap.get(CRServo::class.java, "leftLift")
        rightLiftServo = hardwareMap.get(CRServo::class.java, "rightLift")
        leftLiftServo.direction = Direction.FORWARD
        rightLiftServo.direction = Direction.FORWARD

        spindexerLight = Light(hardwareMap.get(Servo::class.java, "prism"))

        drive = Drive(lf, lb, rf, rb)
        intake = Intake(intakeServo)
        turret = Turret(limelight, turretMotor, flywheelMotor)
        spindexer = Spindexer(
            spindexerServo,
            spindexerDistance,
            spindexerColor,
            spindexerLight,
            spindexerMagnet
        )

        lift = Lift(hardwareMap.get(CRServo::class.java, "leftlift"), hardwareMap.get(CRServo::class.java, "rightlift"))

        updateables = arrayOf(spindexer, turret)

        gamepadState1 = GamepadState()
        lastGamepadState1 = GamepadState()
        gamepadState2 = GamepadState()
        lastGamepadState2 = GamepadState()
    }

    fun update() {
        updateables.forEach { it.update() }

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