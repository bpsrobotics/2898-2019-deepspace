package com.team2898.robot.config

import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.row

val Arm_A = Matrix(arrayOf(row(1.0, 0.00522286), row(0.0, 0.22811787)))
val Arm_B = Matrix(arrayOf(row(0.00119267), row(0.192770918)))
val Arm_Kc = Matrix(arrayOf(row(25.29673028, 1.27854674)))
val Arm_Kff = Matrix(arrayOf(row(0.80116395, 5.17683707)))
val Arm_M = Matrix(arrayOf(row(0.79374646), row(0.0825426)))
const val ARM_OFFSET = 1.1567

const val ARM_KF = 1.3

const val cargol1 = 0.01631882850614698
const val cargol2 = 0.98
const val cargol3 = 1.8805050025612902
const val hatchl1 = 0.0
const val hatchl2 = 0.85
const val hatchl3 = 1.79
const val cargo = 0.6

