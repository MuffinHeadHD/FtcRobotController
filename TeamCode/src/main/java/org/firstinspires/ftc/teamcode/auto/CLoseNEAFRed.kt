package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.AutonomousRobot
import org.firstinspires.ftc.teamcode.parts.TurretConfig
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.PI
import kotlin.math.tan

@Autonomous(name = "Red Close NEAF")
class CLoseNEAFRed : LinearOpMode() {
    lateinit var robot: AutonomousRobot
    val launchPosition = Pose2d(-25.24, 19.13, Math.toRadians(135.0))
    val loadZone = Pose2d(0.0, -59.10,Math.toRadians(89.5))

    override fun runOpMode() {
        robot = AutonomousRobot(this)
        val drive = MecanumDrive(hardwareMap, launchPosition)

        fun setPipeline(index: Int) {
            robot.turret.limelight.pipelineSwitch(index)
            robot.turret.limelight.start()
            sleep(700)
        }

        robot.turret.startLimelight()
        robot.turret.aimTagId = null
        setPipeline(2)

        var motifTag: Int? = null
        var lastId: Int? = null
        var stable = 0

        while (!isStarted && !isStopRequested) {

            val result = robot.turret.limelight.getLatestResult()
            val fid = result?.fiducialResults ?: emptyList()

            val best = fid
                .filter { it.fiducialId == 21 || it.fiducialId == 22 || it.fiducialId == 23 }
                .maxByOrNull { it.targetArea }

            val id = best?.fiducialId
            if (id != null) {
                stable = if (id == lastId) stable + 1 else 1
                lastId = id
                if (stable >= 3) motifTag = id
            }

            telemetry.addData("Prestart pipeline", 2)
            telemetry.addData("MotifTag", motifTag)
            telemetry.addData("Seen IDs", fid.joinToString { it.fiducialId.toString() })
            telemetry.update()
        }

        waitForStart()
        if (isStopRequested) return

        robot.spindexer.home()

        robot.turret.aimTagId = null
        setPipeline(0)

        var seenGoalStable = 0
        var sawGoalEver = false
        val acquireStart = System.currentTimeMillis()

        while (opModeIsActive() && System.currentTimeMillis() - acquireStart < 2500) {
            robot.update()

            val result = robot.turret.limelight.getLatestResult()
            val fid = result?.fiducialResults ?: emptyList()
            val seenIds = fid.map { it.fiducialId }
            val goal = fid.firstOrNull { it.fiducialId == 24 }

            val seenGoal = goal != null
            if (seenGoal) {
                sawGoalEver = true
                seenGoalStable++
            } else {
                seenGoalStable = 0
            }

            telemetry.addData("Goal pipeline", 0)
            telemetry.addData("Result valid", result?.isValid)
            telemetry.addData("Seen IDs", if (seenIds.isEmpty()) "none" else seenIds.joinToString())
            telemetry.addData("GoalTagSeen", seenGoal)
            telemetry.addData("Goal tx", goal?.targetXDegrees)
            telemetry.addData("Goal ty", goal?.targetYDegrees)
            val tag = robot.turret.limelight.getLatestResult().fiducialResults.firstOrNull()
            if (tag != null) {
                telemetry.addData(
                    "Limelight Distance",
                    (robot.turret.tagHeightCm - robot.turret.camHeightCm) / tan(tag.targetYDegrees)
                )
            }
            telemetry.addData("SeenGoalStable", seenGoalStable)
            telemetry.update()

            if (seenGoalStable >= 2) break
        }

        robot.turret.aimTagId = 24

        fun cw() = SequentialAction(robot.SpindexerRotate(+1), SleepAction(1.35))
        fun ccw() = SequentialAction(robot.SpindexerRotate(-1), SleepAction(1.35))

        fun makeLaunchAll() = RaceAction(
            robot.Update(),
            SequentialAction(
                SleepAction(1.0),
                when (motifTag) {
                    else -> SequentialAction(cw(), cw(), cw())
                }
            )
        )

        val oldDelta1 = TurretConfig.flywheelVelocityDelta
        TurretConfig.flywheelVelocityDelta = oldDelta1
        TurretConfig.flywheelVelocityDelta = oldDelta1



        val action = RaceAction(
            robot.Update(),
            drive.actionBuilder(launchPosition)
                .stopAndAdd(SleepAction(0.3))
                .stopAndAdd(robot.SetIntakeIn())
                .build()
        )
        runBlocking(action)

        val oldDelta2 = TurretConfig.flywheelVelocityDelta
        TurretConfig.flywheelVelocityDelta = oldDelta2
        runBlocking(makeLaunchAll())
        TurretConfig.flywheelVelocityDelta = oldDelta2

        runBlocking(
            RaceAction(
                robot.Update(),
                drive.actionBuilder(launchPosition)
                    .stopAndAdd(SleepAction(0.3))
                    .splineToLinearHeading(loadZone, PI / 2)
                    .stopAndAdd(SleepAction(0.75))
                    .stopAndAdd(robot.SpindexerRotate(-1))
                    .stopAndAdd(SleepAction(1.0))
                    .stopAndAdd(robot.SpindexerRotate(-1))
                    .stopAndAdd(SleepAction(1.0))
                    .stopAndAdd(robot.SpindexerRotate(-1))
                    .stopAndAdd(SleepAction(1.0))
                    .splineToLinearHeading(launchPosition, PI /2)
                    .stopAndAdd(robot.SetIntakeIn())
                    .build()
            )
        )
        runBlocking(action)

        val oldDelta3 = TurretConfig.flywheelVelocityDelta
        TurretConfig.flywheelVelocityDelta = oldDelta3
        runBlocking(makeLaunchAll())
        TurretConfig.flywheelVelocityDelta = oldDelta3
    }
}