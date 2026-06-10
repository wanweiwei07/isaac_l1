"""Open the converted l1_h0602 USD stage in the Isaac Sim GUI."""
import os

from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "usd", "l1_h0602.usda")

simulation_app = SimulationApp({"headless": False}) 

import omni.usd

omni.usd.get_context().open_stage(USD_PATH)
print(f"Opened stage: {USD_PATH}")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
