from dataclasses import dataclass

@dataclass
class SimulationParameters:
    host: str = '127.0.0.1'
    port: int = 12345
    Nsim: int = 5
    trigger_end: int = 0
    dim_test_vec: int = 7
    nested_idx: int = 0
    loop_idx: int = 5
