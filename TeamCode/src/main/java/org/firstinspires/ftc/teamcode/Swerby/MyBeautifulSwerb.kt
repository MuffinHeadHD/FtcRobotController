/*
package org.firstinspires.ftc.teamcode.Swerby

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.parts.Updatable


@Config
class SwerbConfig {
    companion object {

        @JvmField
         var speedFactor = 1.0

    }
}
@Config
class SwerbRightPIDConfig {
    companion object{
        @JvmField
        var right_kp = 0.0
        @JvmField
        var right_ki = 0.0
        @JvmField
        var right_kd = 0.0
        @JvmField
        var right_ks = 0.0
        @JvmField
        var right_kv = 0.0
    }
}

@Config
class SwerbLeftPIDConfig {
    companion object {
        @JvmField
        var left_kp = 0.0

        @JvmField
        var left_ki = 0.0

        @JvmField
        var left_kd = 0.0

        @JvmField
        var left_ks = 0.0

        @JvmField
        var left_kv = 0.0
    }
}

class SwerveDrive(val leftTop: DcMotor, val leftBottom: DcMotor, val rightTop: DcMotor, val rightBottom: DcMotor, val leftPodEncoder: DcMotorEx, val rightPodEncoder: DcMotorEx) : Updatable {

    init {
        leftTop.setDirection(DcMotorSimple.Direction.FORWARD)
        leftBottom.setDirection(DcMotorSimple.Direction.REVERSE)
        rightTop.setDirection(DcMotorSimple.Direction.REVERSE)
        rightBottom.setDirection(DcMotorSimple.Direction.REVERSE)

        leftTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightTop.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightBottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER


        leftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        leftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        rightTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        rightBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
    }



    // variables go here
    //-------------------------------------------------------------------

    private var homing = false
    private var homePower = 0.5

    private var targetPower = 0.6


    //-------------------------------------------------------------------

    var dashboard: FtcDashboard = FtcDashboard.getInstance()

    var dashboardTelemetry: Telemetry = dashboard.telemetry


    private var leftPodAngle = leftPodEncoder.currentPosition.toDouble()
    private var rightPodAngle = rightPodEncoder.currentPosition.toDouble()

    fun RightPodAnglePID() {
            var rightlastError = 0.0
            var rightintegral = 0.0
            var rightlastTime = System.nanoTime()

            fun right_Calculation(right_error: Double): Double {
                val right_now = System.nanoTime()
                val right_dt = (right_now - rightlastTime) / 1e9
                rightlastTime = right_now

                rightintegral += right_error * right_dt
                val rightDerivative = (error - rightlastError) / right_dt
                rightlastError = error

                return SwerbRightPIDConfig.right_kP * error + SwerbRightPIDConfig.right_kI * rightintegral + SwerbRightPIDConfig.right_kD * rightDerivative
            }
        }

    }


    fun leftPodPower(topPower: Double, bottomPower: Double) {
        if (homing) {
            leftPodPower(homePower, homePower)
            } else {
                leftPodPower(left_targetPower, left_targetPower)
        }
    }


    fun HomeBiased(offset: Double = 5.0) {
        homing = true
        LeftPodAngleTarget = offset
        RightPodAngleTarget = offset
    }

    fun Home() {
        HomeBiased(0)
    }

    fun leftPodPower(topPower: Double, bottomPower: Double) {
        if (homing) {
            leftPodPower(homePower, homePower)
        } else {
            leftPodPower(targetPower, targetPower)
        }
    }

    override fun update() {
        TODO("Not yet implemented")
    }

}
        */