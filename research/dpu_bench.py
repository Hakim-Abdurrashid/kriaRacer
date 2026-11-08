"""Run a compiled .xmodel on the Kria DPU with VART and report latency (RACE_PLAN.md §5.4).

usage (overlay must be loaded):  sudo bash -c 'source /etc/pynq_venv.sh && python3 dpu_bench.py model.xmodel [n]'
"""
import sys, time
import numpy as np
import vart, xir

g = xir.Graph.deserialize(sys.argv[1])
subs = [s for s in g.get_root_subgraph().toposort_child_subgraph()
        if s.has_attr("device") and s.get_attr("device").upper() == "DPU"]
print(f"DPU subgraphs: {len(subs)}  fingerprint in model: {hex(subs[0].get_attr('dpu_fingerprint')) if subs[0].has_attr('dpu_fingerprint') else 'n/a'}")
r = vart.Runner.create_runner(subs[0], "run")
ins, outs = r.get_input_tensors(), r.get_output_tensors()
for t in ins + outs:
    print(f"  {'in ' if t in ins else 'out'} {t.name:60s} {tuple(t.dims)} fix_point={t.get_attr('fix_point')}")
inp = [np.random.randint(-128, 127, size=tuple(t.dims), dtype=np.int8) for t in ins]
out = [np.empty(tuple(t.dims), dtype=np.int8) for t in outs]
for _ in range(10):
    r.wait(r.execute_async(inp, out))
n = int(sys.argv[2]) if len(sys.argv) > 2 else 300
lat = []
for _ in range(n):
    t0 = time.perf_counter(); r.wait(r.execute_async(inp, out)); lat.append((time.perf_counter() - t0) * 1e3)
lat = np.sort(lat)
print(f"DPU latency over {n} runs: p50 {lat[n//2]:.2f} ms, p99 {lat[int(n*0.99)]:.2f} ms, max {lat[-1]:.2f} ms -> {1000/lat[n//2]:.0f} FPS")
