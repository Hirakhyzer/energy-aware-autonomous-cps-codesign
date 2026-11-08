"""ECO-CPS: simulation-first sensing/computation/control energy co-design."""
from .benchmark import POLICIES, benchmark, summarize, paired_analysis
from .models import EnergyLedger, OperatingMode, RunResult, Scenario
from .policies import choose_mode, feasibility_reasons
from .simulation import simulate

__all__ = ["EnergyLedger", "OperatingMode", "RunResult", "Scenario", "POLICIES",
           "simulate", "benchmark", "summarize", "paired_analysis", "choose_mode", "feasibility_reasons"]
__version__ = "0.2.0"
