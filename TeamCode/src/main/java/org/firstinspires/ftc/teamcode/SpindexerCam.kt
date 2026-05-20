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

@Config

class SpindexerCam(val servo: AxonServo): Updatable {

    public var lastRotateTimeMs = 0L

    var kp = 1.5 // just in case I fuck smth up, the #s are 1.5, 0.075, 0.0, 0.035, 0.0

    var ki = 0.075

    var kd = 0.0

    var ks = 0.035

    var kv = 0.0

    var lastSpinTime = 500L

    private var homePosition: Double = 0.7554545454545455

    override fun update() {
        servo.update()
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
}