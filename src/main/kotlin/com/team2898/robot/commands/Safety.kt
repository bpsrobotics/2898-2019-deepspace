package com.team2898.robot.commands

import com.team2898.robot.subsystem.Arm
import edu.wpi.first.wpilibj.command.InstantCommand

object Safety : InstantCommand({ Arm.startZeroing() })
