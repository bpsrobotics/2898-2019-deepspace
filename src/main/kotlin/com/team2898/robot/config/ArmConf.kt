package com.team2898.robot.config.ArmConf

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.row

val Arm_A = Matrix(arrayOf(row(1.0, 0.01244403), row(0.0, 0.35417244)))
val Arm_B = Matrix(arrayOf(row(0.00318336), row(0.27208949)))
val Arm_Kc = Matrix(arrayOf(row(17.49565796, 1.4460536)))
val Arm_Kff = Matrix(arrayOf(row(1.07079958, 3.66011922)))
val Arm_M = Matrix(arrayOf(row(0.79605582), row(0.33683546)))
val ARM_OFFSET = - 1.0376855067732285

val ARM_KF = 4.0