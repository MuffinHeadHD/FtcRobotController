package org.firstinspires.ftc.teamcode.Swerby

class SwerbPID(
    private val kP: Double,
    private val kI: Double,
    private val kD: Double,
) {
    private var lastError = 0.0
    private var integral = 0.0
    private var lastTime = System.nanoTime()

    fun update(error: Double): Double {
        val now = System.nanoTime()
        val dt = (now - lastTime) / 1e9
        lastTime = now

        integral += error * dt
        val derivative = (error - lastError) / dt
        lastError = error

        return kP * error + kI * integral + kD * derivative
    }
}
