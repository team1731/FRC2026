package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.turret.TurretSubsystemAI;

public class GoalTrackingCommand extends Command {
     private final TurretSubsystemAI m_leftTurret;
   private final TurretSubsystemAI m_rightTurret;

    public GoalTrackingCommand(TurretSubsystemAI left, TurretSubsystemAI right) {
         m_leftTurret = left;
       m_rightTurret = right;
        // Require both so no other command moves them individually
        addRequirements(m_rightTurret, m_leftTurret);
    }

    @Override
    public void initialize() {
      //    m_leftTurret.setTrackingEnabled(true);
      //  m_rightTurret.setTrackingEnabled(true);
    }

    @Override
    public void end(boolean interrupted) {
    //  m_leftTurret.setTrackingEnabled(false);
    //    m_rightTurret.setTrackingEnabled(false);
    }
}
