package org.firstinspires.ftc.teamcode.parts

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.usb.FakeSerialNumber
import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.PIDValues
import org.firstinspires.ftc.teamcode.parts.Turret

enum class LightColor(val index: Int) {
    RED(0),
    PURPLE(5),
    GREEN(2)
}

@Config
class SpindexerPID {
    companion object {
        @JvmField
        var kp = 1.5 // just in case I fuck smth up, the #s are 1.5, 0.075, 0.0, 0.035, 0.0
        @JvmField
        var ki = 0.075
        @JvmField
        var kd = 0.0
        @JvmField
        var ks = 0.035
        @JvmField
        var kv = 0.0
        @JvmField
        var lastSpinTime = 500L
    }
}

class Spindexer(val servo: AxonServo, val distanceSensor: DistanceSensor, val colorSensorFront: ColorSensor, val colorSensorLeft: ColorSensor, val colorSensorRight: ColorSensor, val light: Light, val touchSensor: TouchSensor, var ballPresentFront: Boolean = false, var ballPresentLeft: Boolean = false, var ballPresentRight: Boolean = false, var SpindexerShootingReady: Boolean = false): Updatable {
    private val prismColorsUs = intArrayOf(1050, 1200, 1350, 1500, 1650, 1800, 1940)
    private val inDistance: Double = 3.4
    private val outDistance: Double = 4.3
    private val inAlpha: Int = 28
    public var lastRotateTimeMs = 0L
    private val outAlpha: Int = 18
    private var hueFilt = 0f

    private var homePosition: Double = 0.7554545454545455

    override fun update() {
        servo.pid.kp = SpindexerPID.kp
        servo.pid.ki = SpindexerPID.ki
        servo.pid.kd = SpindexerPID.kd
        servo.pid.ks = SpindexerPID.ks
        servo.pid.kv = SpindexerPID.kv

        servo.update()

        checkBallFront()
        checkBallLeft()
        checkBallRight()

        if (ballPresentFront && ballPresentRight && ballPresentLeft) {
            SpindexerShootingReady = true
        }
        if (!ballPresentFront && !ballPresentRight && !ballPresentLeft) {
            SpindexerShootingReady = false
        }
    }

    fun checkBallFront() {
        val r = colorSensorFront.red()
        val g = colorSensorFront.green()
        val b = colorSensorFront.blue()
        val a = colorSensorFront.alpha()

        val hsvFront = FloatArray(3)
        Color.RGBToHSV(r, g, b, hsvFront)

        val distance = distanceSensor.getDistance(DistanceUnit.CM)
        val distanceValid = distance.isFinite() && distance > 0.0

        ballPresentFront = if (ballPresentFront) {
            (distanceValid && distance < inDistance) && (a > inAlpha)
        } else {
            !((!distanceValid || distance > outDistance) || a < outAlpha)
        }
    }

    fun checkBallRight() {
        val r = colorSensorRight.red()
        val g = colorSensorRight.green()
        val b = colorSensorRight.blue()
        val a = colorSensorRight.alpha()

        val hsvRight = FloatArray(3)
        Color.RGBToHSV(r, g, b, hsvRight)

        val distance = distanceSensor.getDistance(DistanceUnit.CM)
        val distanceValid = distance.isFinite() && distance > 0.0

        ballPresentRight = if (ballPresentRight) {
            (distanceValid && distance < inDistance) && (a > inAlpha)
        } else {
            !((!distanceValid || distance > outDistance) || a < outAlpha)
        }

        val hueValid = ballPresentFront && a >= inAlpha && hsvRight[1] >= 0.12f && hsvRight[2] >= 0.12f
        hueFilt = if (hueValid) 0.8f * hueFilt + 0.2f * hsvRight[0] else hsvRight[0]

        if (!hueValid) {
            setLightColor(LightColor.RED)
        } else if (hueFilt >= 190f) {
            setLightColor(LightColor.PURPLE)
        } else {
            setLightColor(LightColor.GREEN)
        }
    }

    fun checkBallLeft() {
        val r = colorSensorLeft.red()
        val g = colorSensorLeft.green()
        val b = colorSensorLeft.blue()
        val a = colorSensorLeft.alpha()

        val hsvLeft = FloatArray(3)
        Color.RGBToHSV(r, g, b, hsvLeft)

        val distance = distanceSensor.getDistance(DistanceUnit.CM)
        val distanceValid = distance.isFinite() && distance > 0.0

        ballPresentLeft = if (ballPresentLeft) {
            (distanceValid && distance < inDistance) && (a > inAlpha)
        } else {
            !((!distanceValid || distance > outDistance) || a < outAlpha)
        }
    }

    fun rotate(rotation: Int) {
        val target = servo.targetPosition + (1.0/3.0 * rotation)
        servo.targetPosition = (target % 1.0 + 1.0) % 1.0
        lastRotateTimeMs = System.currentTimeMillis()
    }

    fun setRotation(rotation: Int) {
        val target = 1.0/3.0 * rotation
        servo.targetPosition = (target % 1.0 + 1.0) % 1.0
        lastRotateTimeMs = System.currentTimeMillis()
    }
    fun wasRecentlyRotated(windowMs: Long = SpindexerPID.lastSpinTime): Boolean {
        return System.currentTimeMillis() - lastRotateTimeMs < windowMs
    }

    fun isFinished(): Boolean {
        return servo.servo.power == 0.0
    }

    fun home() {
        servo.targetPosition = homePosition
        lastRotateTimeMs = System.currentTimeMillis()
    }

    fun setLightColor(color: LightColor) {
        light.setUs(prismColorsUs[color.index])
    }
}