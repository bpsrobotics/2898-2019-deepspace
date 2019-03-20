package com.team2898.robot.commands.commandGroups

import com.team2898.robot.commands.Safety
import com.team2898.robot.commands.Teleop
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup

object TeleopWSafety: CommandGroup() {
    init {
        addSequential(Safety)
        addSequential(Teleop)
    }
}