package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.AutonomousRobot
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.PI
import kotlin.math.tan

@Autonomous(name = "BasicBlueFar")
class BasicBlue : LinearOpMode() {
    lateinit var robot: AutonomousRobot
    val initialPose = Pose2d(64.0, -15.0, Math.toRadians(180.0))
    val endPose = Pose2d(initialPose.position.x, initialPose.position.y, Math.toRadians(180.0))

    override fun runOpMode() {
        robot = AutonomousRobot(this)
        val drive = MecanumDrive(hardwareMap, initialPose)

        fun setPipeline(index: Int) {
            robot.turret.limelight.pipelineSwitch(index)
            robot.turret.limelight.start()
            sleep(700) // give pipeline more time to settle
        }

        robot.turret.startLimelight()
        robot.turret.aimTagId = null
        setPipeline(2)

        var motifTag: Int? = null
        var lastId: Int? = null
        var stable = 0

        while (!isStarted && !isStopRequested) {
            robot.update()

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

        // switch to goal pipeline
        robot.turret.aimTagId = null
        setPipeline(0)

        // debug goal detection BEFORE enabling aim
        var seenGoalStable = 0
        var sawGoalEver = false
        val acquireStart = System.currentTimeMillis()

        while (opModeIsActive() && System.currentTimeMillis() - acquireStart < 2500) {
            robot.update()

            val result = robot.turret.limelight.getLatestResult()
            val fid = result?.fiducialResults ?: emptyList()
            val seenIds = fid.map { it.fiducialId }
            val goal = fid.firstOrNull { it.fiducialId == 20 }

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

        // only enable auto-aim after confirming tag is actually visible
        robot.turret.aimTagId = 20

        fun cw() = SequentialAction(robot.SpindexerRotate(+1), SleepAction(1.5))
        fun ccw() = SequentialAction(robot.SpindexerRotate(-1), SleepAction(1.5))

        fun makeLaunchAll() = RaceAction(
            robot.Update(),
            SequentialAction(
                SleepAction(3.0),
                when (motifTag) {
                    22 -> SequentialAction(cw(), cw(), cw())
                    21 -> SequentialAction(ccw(), ccw(), cw(), cw(), cw())
                    23 -> SequentialAction(ccw(), cw(), cw(), cw())
                    else -> SequentialAction(cw(), cw(), cw())
                }
            )
        )

        runBlocking(makeLaunchAll())

        val action = RaceAction(
            robot.Update(),
            drive.actionBuilder(initialPose)
                .stopAndAdd(
                    SequentialAction(
                        InstantAction { robot.turret.aimTagId = null },
                        InstantAction { robot.turret.home() },
                        SleepAction(0.2)
                    )
                )
                .stopAndAdd(robot.SetIntakeIn())
                .splineTo(Vector2d(37.0, -30.0), Math.toRadians(-90.0))
                .turnTo(Math.toRadians(-90.0))
                .stopAndAdd(SleepAction(0.5))
                .splineTo(Vector2d(37.0, -37.0), Math.toRadians(-90.0))
                .turnTo(Math.toRadians(-90.0))
                .stopAndAdd(robot.SpindexerRotate(-1))
                .stopAndAdd(SleepAction(0.35))
                .splineTo(Vector2d(37.0, -41.0), Math.toRadians(-90.0))
                .turnTo(Math.toRadians(-90.0))
                .stopAndAdd(SleepAction(0.5))
                .stopAndAdd(robot.SpindexerRotate(-1))
                .stopAndAdd(SleepAction(0.35))
                .splineTo(Vector2d(37.0, -44.0), Math.toRadians(-90.0))
                .turnTo(Math.toRadians(-90.0))
                .stopAndAdd(SleepAction(0.5))
                .splineToLinearHeading(initialPose, PI / 2)
                .stopAndAdd(
                    SequentialAction(
                        InstantAction { robot.turret.aimTagId = null },
                        InstantAction { robot.turret.home() },
                        SleepAction(0.2)
                    )
                )
                .strafeTo(Vector2d(initialPose.position.x, initialPose.position.y))
                .build()
        )

        runBlocking(action)
        runBlocking(makeLaunchAll())
        runBlocking(
            RaceAction(
                robot.Update(),
                drive.actionBuilder(endPose)
                    .strafeTo(Vector2d(endPose.position.x, endPose.position.y - 20.0))
                    .build()
            )
        )
    }
}