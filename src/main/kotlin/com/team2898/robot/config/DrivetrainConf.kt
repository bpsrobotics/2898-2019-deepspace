package com.team2898.robot.config.DrivetrainConf

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.row

val Dt_A = Matrix(arrayOf(
        row(-6.5968, -3.1471),
        row(-3.1471, -6.5968)
))
val Dt_B = Matrix(arrayOf(
        row(2.4971, 1.1913),
        row(1.1913, 2.4971)
))

val Dt_Kc = Matrix(arrayOf(
        row(8.744, -0.6838),
        row(-0.6838, 8.744)
))

val Dt_Kff = Matrix(arrayOf(
        row(53.0104, -24.5718),
        row(-24.5718, 53.0104)
))

val Dt_M = Matrix(Matrix(arrayOf(
        row(0.5419, 0.0029),
        row(0.0029, 0.5419)
)).scalarMultiply(1e-6).data)
