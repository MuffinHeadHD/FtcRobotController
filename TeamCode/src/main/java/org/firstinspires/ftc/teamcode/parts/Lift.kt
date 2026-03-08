package org.firstinspires.ftc.teamcode.parts

import com.qualcomm.robotcore.hardware.CRServo

class Lift(val leftLift: CRServo, val rightLift: CRServo) {
    fun setPower(p: Double) {
        val power = p.coerceIn(-1.0, 1.0)
        leftLift.power = power
        rightLift.power = power
    }

    fun up() {
        setPower(1.0)
    }

    fun down() {
        setPower(-1.0)
    }

    fun stop() {
        setPower(0.0)
    }
}