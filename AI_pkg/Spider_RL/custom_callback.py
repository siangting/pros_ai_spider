from stable_baselines3.common.callbacks import BaseCallback
import datetime


# Save model
class CustomCallback(BaseCallback):
    def __init__(self, save_path: str, save_freq: int, verbose = 0):
        super(CustomCallback, self).__init__(verbose)
        self.save_path = save_path
        self.save_freq = save_freq
        self.last_save_step = 0

    def _on_step(self):
        """
        Saving function schedually every "save_freq" steps.
        """
        if self.n_calls - self.last_save_step >= self.save_freq:
            print(f"Run {self.n_calls} steps...")
            self.date = datetime.date.today()
            self.model.save(f"{self.save_path}_{self.date}.pt")
            print(f"Save {self.save_path}_{self.date}.pt\n\n")
            self.last_save_step = self.n_calls

        return True
