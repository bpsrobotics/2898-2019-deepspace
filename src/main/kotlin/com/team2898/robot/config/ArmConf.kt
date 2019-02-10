package com.team2898.robot.config.ArmConf

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.row

val Arm_A = Matrix(arrayOf(row(1.0, 9.715e-4), row(0.0, 9.435e-1)))
val Arm_B = Matrix(arrayOf(row(1.2e-5), row(2.377e02)))
val Arm_Kc = Matrix(arrayOf(row(193.97, 36.63)))
val Arm_Kff = Matrix(arrayOf(row(0.485, 38.46)))
val Arm_M = Matrix(arrayOf(row(0.7934), row(0.648)))