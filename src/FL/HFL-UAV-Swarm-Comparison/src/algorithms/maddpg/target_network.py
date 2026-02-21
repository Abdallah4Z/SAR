import torch

class TargetNetwork:
    """
    Manages soft updates for target networks.
    """
    def __init__(self, target_model, source_model, tau):
        self.target_model = target_model
        self.source_model = source_model
        self.tau = tau
        
        # Initialize target weights to match source
        self.hard_update()

    def soft_update(self):
        """
        Soft update model parameters.
        θ_target = τ*θ_local + (1 - τ)*θ_target
        """
        for target_param, param in zip(self.target_model.parameters(), self.source_model.parameters()):
            target_param.data.copy_(
                self.tau * param.data + (1.0 - self.tau) * target_param.data
            )

    def hard_update(self):
        """
        Hard update model parameters.
        θ_target = θ_local
        """
        for target_param, param in zip(self.target_model.parameters(), self.source_model.parameters()):
            target_param.data.copy_(param.data)
