package com.team2898.robot.subsystem

import com.team2898.engine.subsystems.DrivetrainLQR
import com.team2898.robot.config.DrivetrainConf.*
import edu.wpi.first.wpilibj.Encoder


object Drivetrain: DrivetrainLQR() {
    override val wheelbase: Double
        get() = 2.0

    override val A = Dt_A
    override val B = Dt_B
    override val Kc = Dt_Kc
    override val Kff = Dt_Kff
    override val M = Dt_M

    val leftEnc = Encoder(0, 1)
    val rightEnc = Encoder(2, 3)

    init {
        listOf(leftEnc, rightEnc).forEach {
            it.apply {
            }
        }
    }
}
