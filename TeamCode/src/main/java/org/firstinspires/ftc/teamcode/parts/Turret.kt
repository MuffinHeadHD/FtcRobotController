package org.firstinspires.ftc.teamcode.parts

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.AutonomousRobot
import org.firstinspires.ftc.teamcode.Robot
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

        @JvmField
        var spikeVelIncrease = 500// spike increase in ticks/sec
        @JvmField
        var spikeDurationMs = 500L // time 1000 = 1 sec
        @JvmField
        var spikeTriggerVolts = 0.20 // idk what this even means anymore, i'm just guessing and hoping it works

        @JvmField
        var  FlywheelLowVoltageAdditive  /* Power */ = 0.0 // this adds velocity when voltage is low (manually)

    }
}

class Turret(val limelight: Limelight3A, val turretMotor: DcMotor, val flywheelMotor: DcMotorEx, val launchDist: AnalogInput, val backlightR: Servo, val backlightL: Servo, val spindexer: Spindexer) : Updatable {

    init {
        turretMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turretMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    var aimTagId: Int? = null

    private var lastTxDeg = 0.0
    private var lastTyDeg = 0.0
    private var lastSeenTimeMs = System.currentTimeMillis()
    private val holdLastSeenMs = 250L

    private val deadbandDeg = 0.5
    private val kP_tt = 0.014
    private val minPower = 0.040
    private val maxPower_tt = 1.0
    private var lastTx = 0.0
    private var txVel = 0.0
    private val kD_tt = 0.0010

    private val camMountDeg = 19.97
    val tagHeightCm = 74.93
    val camHeightCm = 36.02493
    private var distFiltCm = 0.0

    private val MaxVoltage = 3.3
    private val MaxDistance_mm = 1000.0

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
    private val kP_vel = 0.00032

    private val PIPELINE_INDEX = 2

    private var homing = false
    private val homePower = 0.50
    private val homeTol = 20



        // ---------------------------------------------------------------------
        private var spiking = false
        private var spikeStartTime = 0L

        var dashboard: FtcDashboard = FtcDashboard.getInstance()

        var dashboardTelemetry: Telemetry = dashboard.telemetry

        // ---------------------------------------------------------------------


    var lastTime = System.nanoTime()

    fun startLimelight() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(PIPELINE_INDEX)
        limelight.start()
    }

    fun homeBiased(offset: Int = 5) {
        homing = true
        turretMotor.targetPosition = offset
        turretMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        turretMotor.power = homePower
    }

    fun home() {
        homeBiased(0)
    }

    override fun update() {
        val now = System.nanoTime()
        val dt = (now - lastTime) / 1e9
        lastTime = now

        val result = limelight.getLatestResult()

        var launchDistVolts = launchDist.getVoltage()
        var distInLauncher = ((launchDistVolts /MaxVoltage) * MaxDistance_mm)

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
            txVel = (txToUse - lastTx) / dt
            lastTx = txToUse
            val turretCmdPower =
                if (abs(txToUse) <= deadbandDeg) {
                    0.0
                } else {
                    var pwr = kP_tt * txToUse - kD_tt * txVel
                    if (abs(pwr) < minPower) pwr = 0.0
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
        val velPts = doubleArrayOf(1690.0, 1870.6459, 1902.0, 2022.901, 2106.0, 2420.0)

    /*
     if (distFresh) {
            val v = interp1D(distFiltCm, distPts, velPts)
            targetVelCmd = v.coerceIn(minVel, maxVel)
        }

     */



        // ---------------------------------------------------------------------



        if (distFresh) {
                val v = interp1D(distFiltCm, distPts, velPts)


                if ((launchDistVolts < TurretConfig.spikeTriggerVolts) && !spiking && spindexer.wasRecentlyRotated()) {
                    spiking = true
                    spikeStartTime = System.currentTimeMillis()
                }

                targetVelCmd = if (spiking) {
                    (v + TurretConfig.spikeVelIncrease).coerceIn(minVel, maxVel)
                } else {
                    v.coerceIn(minVel, maxVel)
                }
            }


        if (spiking) {
            val elapsed = System.currentTimeMillis() - spikeStartTime
            if (elapsed >= TurretConfig.spikeDurationMs) {
                spiking = false
            }
        }


        dashboardTelemetry.addData("dist volts", launchDistVolts)
        dashboardTelemetry.addData("Spiking", spiking)
        dashboardTelemetry.addData("Distance of Sensor in launcher", distInLauncher)
        dashboardTelemetry.addData("recentRotate", spindexer.wasRecentlyRotated())
        dashboardTelemetry.addData("lastRotateMs", spindexer.lastRotateTimeMs)
        dashboardTelemetry.update()

        // ---------------------------------------------------------------------

        targetVel = slew(targetVel, targetVelCmd, dt, velSlew)
        val measuredVel = abs(flywheelMotor.velocity)
        val ff = (targetVel / maxVelEst).coerceIn(0.0, 1.0)
        val err = targetVel - measuredVel
        val corr = kP_vel * err
        val out = (ff + corr + TurretConfig.FlywheelLowVoltageAdditive).coerceIn(0.0, 1.0)

        val RED = 0.28
        val YELLOW = 0.34
        val GREEN = 0.50
        val REDOVER = 0.28

        val error = targetVel - measuredVel

        val greenThreshold = 600.0  //green
        val yellowThreshold = 1450.0  //yellow
        val blinkThreshold = 2850.0   // red blink

        val blinkPeriod = 500L //.5 sec
        val blinkOn = (System.currentTimeMillis() / blinkPeriod) % 2 == 0L

        val colorPos = when {

            measuredVel + 500.0 > targetVel -> {
                if (blinkOn) REDOVER else 0.0
            }
            error > blinkThreshold -> {
                if (blinkOn) RED else 0.0
            }
            error > yellowThreshold -> RED
            error > greenThreshold -> YELLOW
            else -> GREEN
        }

        backlightR.setPosition(colorPos)
        backlightL.setPosition(colorPos)                                                                                                                                                                                                                                                                                                                                                      //chicken

        flyPower = rampTowards(flyPower, out, powerSlewPerSec * dt)
        flywheelMotor.power = -flyPower
    }
}