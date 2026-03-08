package org.firstinspires.ftc.teamcode.parts

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.clamp
import org.firstinspires.ftc.teamcode.interp1D
import org.firstinspires.ftc.teamcode.rampTowards
import org.firstinspires.ftc.teamcode.slew
import kotlin.math.abs
import kotlin.math.tan

@Config
class TurretConfig {
    companion object {
        @JvmField
        var flywheelVelocityDelta = 0.0
    }
}

class Turret(val limelight: Limelight3A, val turretMotor: DcMotor, val flywheelMotor: DcMotorEx) : Updatable {

    init {
        turretMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turretMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    var aimTagId: Int? = null

    private var lastTxDeg = 0.0
    private var lastTyDeg = 0.0
    private var lastSeenTimeMs = System.currentTimeMillis()
    private val holdLastSeenMs = 250L

    private val deadbandDeg = 1.0
    private val kP_tt = 0.02
    private val minPower = 0.012
    private val maxPower_tt = 1.0

    private val camMountDeg = 20.0
    val tagHeightCm = 74.93
    val camHeightCm = 36.02493
    private var distFiltCm = 0.0
    private val distAlpha = 0.25
    private var lastDistSeenMs = System.currentTimeMillis()
    private val holdDistMs = 250L

    private val minVel = 0.0
    private val maxVel = 4500.0
    private var targetVelCmd = 2200.0
    private var targetVel = 2200.0
    private val velSlew = 7000.0
    private val powerSlewPerSec = 5.0
    var flyPower = 0.0
    private val maxVelEst = 4000.0
    private val kP_vel = 0.0003

    private val PIPELINE_INDEX = 2

    private var homing = false
    private val homePower = 0.55
    private val homeTol = 15

    var lastTime = System.nanoTime()

    fun startLimelight() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(PIPELINE_INDEX)
        limelight.start()
    }

    fun home() {
        homing = true
        turretMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        turretMotor.targetPosition = 0
        turretMotor.power = homePower
    }

    override fun update() {
        val now = System.nanoTime()
        val dt = (now - lastTime) / 1e9
        lastTime = now

        val result = limelight.getLatestResult()

        var hasTargetNow = false
        var txDeg = 0.0
        var tyDeg = 0.0

        if (result != null && result.isValid) {
            val fid = result.fiducialResults

            val best: LLResultTypes.FiducialResult? =
                if (!fid.isNullOrEmpty()) {
                    if (aimTagId != null) {
                        fid.filter { it.fiducialId == aimTagId }.maxByOrNull { it.targetArea }
                    } else {
                        fid.maxByOrNull { it.targetArea }
                    }
                } else {
                    null
                }

            if (best != null) {
                hasTargetNow = true
                txDeg = best.targetXDegrees
                tyDeg = best.targetYDegrees
                lastTxDeg = txDeg
                lastTyDeg = tyDeg
                lastSeenTimeMs = System.currentTimeMillis()
            } else if (aimTagId == null) {
                hasTargetNow = true
                txDeg = result.tx
                tyDeg = result.ty
                lastTxDeg = txDeg
                lastTyDeg = tyDeg
                lastSeenTimeMs = System.currentTimeMillis()
            }
        }

        val nowMs = System.currentTimeMillis()
        val recentlySeen = (nowMs - lastSeenTimeMs) <= holdLastSeenMs

        if (homing) {
            val errHome = turretMotor.targetPosition - turretMotor.currentPosition
            if (abs(errHome) <= homeTol) {
                homing = false
                turretMotor.power = 0.0
                turretMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            } else {
                turretMotor.power = homePower
            }
        } else {
            val txToUse = when {
                hasTargetNow -> txDeg
                recentlySeen -> lastTxDeg
                else -> 0.0
            }

            val turretCmdPower =
                if (abs(txToUse) <= deadbandDeg) {
                    0.0
                } else {
                    var pwr = kP_tt * txToUse
                    pwr = if (pwr > 0) pwr + minPower else pwr - minPower
                    clamp(pwr,-maxPower_tt, maxPower_tt)
                }

            turretMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            turretMotor.power = turretCmdPower
        }

        if (hasTargetNow) {
            val angleRad = Math.toRadians(camMountDeg + tyDeg)
            val tanVal = tan(angleRad)
            if (tanVal > 0.01) {
                val d = (tagHeightCm - camHeightCm) / tanVal
                if (d.isFinite() && d > 0.0) {
                    distFiltCm = if ((nowMs - lastDistSeenMs) > holdDistMs || distFiltCm == 0.0) {
                        d
                    } else {
                        (1.0 - distAlpha) * distFiltCm + distAlpha * d
                    }
                    lastDistSeenMs = nowMs
                }
            }
        }

        val distFresh = (nowMs - lastDistSeenMs) <= holdDistMs

        val distPts = doubleArrayOf(80.0, 120.0, 160.0, 180.0, 200.0, 320.0)
        val velPts = doubleArrayOf(1667.0, 1843.6459, 1879.0, 2007.901, 2080.0, 2510.0)

        if (distFresh) {
            val v = interp1D(distFiltCm, distPts, velPts)
            targetVelCmd = v.coerceIn(minVel, maxVel)
        }

        targetVel = slew(targetVel, targetVelCmd, dt, velSlew)
        val measuredVel = abs(flywheelMotor.velocity)

        val ff = (targetVel / maxVelEst).coerceIn(0.0, 1.0)
        val err = targetVel - measuredVel
        val corr = kP_vel * err
        val out = (ff + corr).coerceIn(0.0, 1.0)

        flyPower = rampTowards(flyPower, out, powerSlewPerSec * dt)
        flywheelMotor.power = -flyPower
    }
}