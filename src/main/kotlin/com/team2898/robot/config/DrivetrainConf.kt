package com.team2898.robot.config.DrivetrainConf

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.row

val Dt_A = Matrix(arrayOf(
        row(0.90634332, -0.05961496),
        row(-0.05961496, 0.90634332)
))

val Dt_B = Matrix(arrayOf(
        row(0.03545732, 0.02256953),
        row(0.02256953, 0.03545732)
))

val Dt_Kc = Matrix(arrayOf(
        row(8.10889379, -1.31590195),
        row(-1.31590195,  8.10889379)
))

//val Dt_Kff = Matrix(arrayOf(
//        row(3.71992519, 1.90743371),
//        row(1.90743371, 3.71992519)
//))

val Dt_Kff = Matrix(arrayOf(
        row(47.41310545, -30.17970195),
        row(-30.17970195, 47.41310545)
))
val Dt_M = Matrix(arrayOf(
        row(9.99999000e-01, -1.08062746e-13),
        row(-1.08062746e-13,  9.99999000e-01)
))
