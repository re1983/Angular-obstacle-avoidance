# Wrapper config that maps to the existing scenario config
import importlib
module = importlib.import_module('monte_carlo_simulation_B737_vs_UAV.config')
globals().update({k: getattr(module, k) for k in dir(module) if not k.startswith('_')})
