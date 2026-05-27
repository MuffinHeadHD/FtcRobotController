package org.firstinspires.ftc.teamcode


import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode
import org.firstinspires.ftc.teamcode.parts.AxonServo
import org.firstinspires.ftc.teamcode.parts.SpindexerCam


@TeleOp(name = "Camera controls")
class CameraController: LinearOpMode() {

        private lateinit var limelight: Limelight3A
        private lateinit var flywheel: DcMotor

        lateinit var spindexer: SpindexerCam
        private lateinit var intake: CRServo
        lateinit var spindexerServo: AxonServo
        lateinit var lf: DcMotor
        lateinit var lb: DcMotor
        lateinit var rf: DcMotor
        lateinit var rb: DcMotor

        private val PIPELINE_INDEX = 0

    enum class CardColor {
            GREEN,
            BLUE,
            RED,
            YELLOW,
            NONE
        }



    override fun runOpMode() {

            limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
            limelight.setPollRateHz(100)
            limelight.pipelineSwitch(0)
            limelight.start()
            flywheel = hardwareMap.get(DcMotor::class.java, "flywheel")
            intake = hardwareMap.get(CRServo::class.java, "intk")
            spindexerServo = AxonServo(
                hardwareMap.get(CRServo::class.java, "spdx"),
                hardwareMap.get(AnalogInput::class.java, "axen")
            )
            spindexer = SpindexerCam(spindexerServo)



            waitForStart()

            while (opModeIsActive()) {

                val color = detectColor()

                val number = readTagNumber()

                when (color) {
                    CardColor.GREEN -> {
                        limelight.pipelineSwitch(4)
                        readTagNumber()
                    }

                    CardColor.BLUE -> {
                        intake.power = -1.0
                        sleep(3000)
                        spindexer.rotate(-1)
                    }

                    CardColor.RED -> {
                        FollowWhenRed()
                    }

                    CardColor.YELLOW -> {
                        intake.power = -1.0
                        spindexer.rotate(1)
                    }

                    CardColor.NONE -> {
                        //leave blank
                    }
                }

                if (number != null) {
                    flywheel.power = number
                }

                telemetry.update()
            }
        }

    fun startLimelight() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(PIPELINE_INDEX)
        limelight.start()
    }

    fun FollowWhenRed() {
        val result = limelight.getLatestResult()

        if (result != null && result.isValid) {
            val tx = result.tx
            val ta = result.ta


            val turn = tx * 0.03
            val targetArea = 5.0
            val drive = (targetArea - ta) * 0.1

            var rightPower = drive + turn
            var leftPower = drive - turn

            rf.power = rightPower
            rb.power = -rightPower

            lf.power = leftPower
            lb.power = -leftPower
        }

    }


    fun readTagNumber(): Double? {
        val result = limelight.getLatestResult()
        if (result == null || !result.isValid) return null

        val tags = result.fiducialResults
        if (tags.isNullOrEmpty()) return null


        val best = tags.maxByOrNull { it.targetArea } ?: return null

        val id = best.fiducialId

        return decodeTagId(id)
    }



    fun decodeTagId(id: Int): Double? {
        return when (id) {
            21 -> 0.0
            22 -> 0.3
            23 -> 0.6
            20 -> 0.8
            24 -> 1.0
            else -> null
        }
    }



    private fun detectColor(): CardColor {

            for (pipeline in 0..3) {
                limelight.pipelineSwitch(pipeline)

                val result = limelight.getLatestResult()

                if (result != null && result.isValid) {
                    return when (pipeline) {
                        0 -> CardColor.GREEN
                        1 -> CardColor.BLUE
                        2 -> CardColor.RED
                        3 -> CardColor.YELLOW
                        else -> CardColor.NONE
                    }
                }
            }

            return CardColor.NONE
        }
    }
