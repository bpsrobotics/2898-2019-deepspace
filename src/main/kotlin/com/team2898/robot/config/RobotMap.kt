package com.team2898.robot.config

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType

// Motor IDs
const val LEFT_MASTER = 2
const val RIGHT_MASTER = 4
const val LEFT_SLAVE = 1
const val RIGHT_SLAVE = 3

const val ARM_RIGHT_MASTER = 5
const val ARM_RIGHT_SLAVE = 6
const val ARM_LEFT_MASTER = 7
const val ARM_LEFT_SLAVE = 8

const val CARGOINTAKE = 9

// DIO
const val SAFETY_LIMIT_PORT = 0

const val ARM_ENC_A = 8
const val ARM_ENC_B = 9

// Pneumatics
data class PneumaticsMap(
    val forward: Int, val reverse: Int,
    val module: Int,
    val modType: PneumaticsModuleType = PneumaticsModuleType.CTREPCM
)

fun DoubleSolenoid(map: PneumaticsMap): DoubleSolenoid = DoubleSolenoid(map.module, map.modType, map.forward, map.reverse)

val HATCH = PneumaticsMap(0, 1, 0)
val LATCH = PneumaticsMap(6, 5, 0)
val SHOOT = PneumaticsMap(0, 1, 1)

val BRAKE = PneumaticsMap(3, 2, 0)
