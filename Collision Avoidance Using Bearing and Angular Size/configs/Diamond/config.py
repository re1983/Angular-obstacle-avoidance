# Wrapper config that maps to the existing scenario config
import importlib
module = importlib.import_module('MCS_diamond.config')
globals().update({k: getattr(module, k) for k in dir(module) if not k.startswith('_')})
