"""Load the DPU-PYNQ B4096 overlay into the PL (run by dpu-overlay.service inside the PYNQ venv)."""
from pynq_dpu import DpuOverlay

DpuOverlay('dpu.bit')
print('DPU overlay loaded')
