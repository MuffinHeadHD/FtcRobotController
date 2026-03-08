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

@Autonomous(name = "BasicBlueFar")
class BasicBlue : LinearOpMode() {
    lateinit var robot: AutonomousRobot
    val initialPose: Pose2d = Pose2d(64.0, -15.0, Math.toRadians(180.0))

    override fun runOpMode() {
        robot = AutonomousRobot(this)
        val drive = MecanumDrive(hardwareMap, initialPose)

        robot.turret.startLimelight()
        robot.turret.aimTagId = null

        var motifTag: Int? = null
        var lastId: Int? = null
        var stable = 0

        while (!isStarted && !isStopRequested) {
            val result = robot.turret.limelight.getLatestResult()
            if (result != null && result.isValid) {
                val fid = result.fiducialResults
                if (!fid.isNullOrEmpty()) {
                    val best = fid
                        .filter { it.fiducialId == 21 || it.fiducialId == 22 || it.fiducialId == 23 }
                        .maxByOrNull { it.targetArea }

                    val id = best?.fiducialId
                    if (id != null) {
                        stable = if (id == lastId) stable + 1 else 1
                        lastId = id
                        if (stable >= 3) motifTag = id
                    }
                }
            }

            telemetry.addData("MotifTag", motifTag)
            telemetry.update()
        }

        waitForStart()
        if (isStopRequested) return

        robot.spindexer.home()
        robot.turret.aimTagId = 20

        fun cw() = SequentialAction(robot.SpindexerRotate(+1), SleepAction(0.8))
        fun ccw() = SequentialAction(robot.SpindexerRotate(-1), SleepAction(0.8))

        runBlocking(
            RaceAction(
                robot.Update(),
                SequentialAction(
                    SleepAction(1.0)
                )
            )
        )

        fun makeLaunchAll() = RaceAction(
            robot.Update(),
            SequentialAction(
                SleepAction(2.0),
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
                .stopAndAdd(robot.SetIntakeIn())
                .splineTo(Vector2d(37.0, -30.0), Math.toRadians(-90.0))
                .turnTo(Math.toRadians(-90.0))
                .stopAndAdd(SleepAction(0.5))
                .splineTo(Vector2d(37.0, -37.0), Math.toRadians(-90.0))
                .turnTo(Math.toRadians(-90.0))
                .stopAndAdd(robot.SpindexerRotate(-1))
                .splineTo(Vector2d(37.0, -41.0), Math.toRadians(-90.0))
                .turnTo(Math.toRadians(-90.0))
                .stopAndAdd(SleepAction(0.5))
                .stopAndAdd(robot.SpindexerRotate(-1))
                .splineTo(Vector2d(37.0, -61.0), Math.toRadians(-90.0))
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
    }
}