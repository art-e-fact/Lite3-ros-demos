#!/usr/bin/env python3
"""Benchmark a pixi integration-test task so runs can be compared across machines.

    pixi run -e nav-test python scripts/bench_sim_test.py test-rerun-recording --label mac-m1pro
"""
import argparse, json, os, platform, re, shutil, socket, statistics, subprocess, sys, threading, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CLASSES = {
    "sim": lambda c: "simulation_package.start_simulation" in c,
    "controller": lambda c: "rl_deploy" in c,
    "nav_nodes": lambda c: any(n in c for n in
                               ("local_heightmap_node", "rail_detector_node", "rail_target_follower_node")),
    "rerun_viewer": lambda c: "--port" in c and os.path.basename(c.split()[0]) == "rerun",
}


def sh(*cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=15).stdout.strip()
    except Exception:
        return ""


def host_info():
    if sys.platform == "darwin":
        cpu = sh("sysctl", "-n", "machdep.cpu.brand_string")
        ram = sh("sysctl", "-n", "hw.memsize")
        ram_gb = round(int(ram) / 2**30, 1) if ram.isdigit() else None
    else:
        cpu = next((l.split(":", 1)[1].strip() for l in Path("/proc/cpuinfo").read_text().splitlines()
                    if l.startswith("model name")), "")
        kb = next((l.split()[1] for l in Path("/proc/meminfo").read_text().splitlines()
                   if l.startswith("MemTotal")), "")
        ram_gb = round(int(kb) / 2**20, 1) if kb.isdigit() else None
    return {"platform": platform.platform(), "python": platform.python_version(), "cpu": cpu,
            "cpu_count": os.cpu_count(), "ram_gb": ram_gb, "hostname": socket.gethostname(),
            "git_sha": sh("git", "-C", str(REPO), "rev-parse", "--short", "HEAD")}


def parse_sim_log(path):
    """Warm-up time and the Warp device list out of a harness sim.log."""
    out = {"warmup_sec": None, "warp": None, "warp_devices": []}
    lines = path.read_text(errors="replace").splitlines()
    for i, line in enumerate(lines):
        if (w := re.search(r"warm-up finished in ([\d.]+) s", line)):
            out["warmup_sec"] = float(w.group(1))
        if (r := re.match(r"^(Warp .*) initialized:", line)):
            out["warp"] = r.group(1)
            for nxt in lines[i + 1:i + 12]:
                if not nxt.startswith(" "):
                    break
                if nxt.strip().startswith('"'):
                    out["warp_devices"].append(" ".join(nxt.split()))
    return out


def sample_loop(stop, proc_samples, gpu_samples):
    """Best-effort once-a-second ps/nvidia-smi sampling; every error is ignored."""
    nvidia = shutil.which("nvidia-smi")
    while not stop.wait(1.0):
        try:
            ps = subprocess.run(["ps", "-axo", "pid,ppid,pcpu,rss,command"],
                                capture_output=True, text=True, timeout=10).stdout
            totals = {k: [0.0, 0.0] for k in CLASSES}
            seen = set()
            for line in ps.splitlines()[1:]:
                parts = line.split(None, 4)
                if len(parts) < 5 or "bench_sim_test" in parts[4]:
                    continue
                for name, match in CLASSES.items():
                    if match(parts[4]):
                        totals[name][0] += float(parts[2])
                        totals[name][1] += float(parts[3]) / 1024
                        seen.add(name)
                        break
            for name in seen:
                proc_samples.setdefault(name, []).append(totals[name])
        except Exception:
            pass
        if nvidia:
            try:
                out = subprocess.run([nvidia, "--query-gpu=name,utilization.gpu,memory.used",
                                      "--format=csv,noheader,nounits"],
                                     capture_output=True, text=True, timeout=10).stdout.strip()
                for row in out.splitlines():
                    name, util, mem = (f.strip() for f in row.split(","))
                    gpu_samples.setdefault(name, []).append((float(util), float(mem)))
            except Exception:
                pass


def summarize(samples, keys):
    out = {}
    for name, rows in samples.items():
        first = [x[0] for x in rows]
        out[name] = dict(zip(keys, (len(rows), statistics.fmean(first), max(first),
                                    max(x[1] for x in rows))))
    return out


def run_task(task, bench_dir):
    env = dict(os.environ)
    addopts = env.get("PYTEST_ADDOPTS", "")
    env["PYTEST_ADDOPTS"] = f"{addopts} --basetemp={bench_dir / 'pytest'}".strip()
    proc_samples, gpu_samples, stop = {}, {}, threading.Event()
    sampler = threading.Thread(target=sample_loop, args=(stop, proc_samples, gpu_samples), daemon=True)
    sampler.start()
    started = time.time()
    with (bench_dir / "task.log").open("w") as log:
        p = subprocess.Popen([os.environ.get("PIXI_EXE", "pixi"), "run", task], cwd=REPO, env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        text = []
        for line in p.stdout:
            sys.stdout.write(line)
            log.write(line)
            text.append(line)
        rc = p.wait()
    stop.set()
    sampler.join(timeout=5)
    return rc, time.time() - started, "".join(text), proc_samples, gpu_samples


def parse_task_output(text):
    def grab(pattern, cast=str):
        m = re.search(pattern, text)
        return cast(m.group(1)) if m else None
    # "simulator ran 52.0 sim s in 150.2 wall s (real-time factor 0.346), 2600 odom messages"
    # is logged by SimControlHarness.stop(); take the first (the module-scoped harness).
    return {"sim_ready_sec": grab(r"simulator ready after ([\d.]+) s", float),
            "sim_span_sec": grab(r"simulator ran ([\d.]+) sim s", float),
            "wall_span_sec": grab(r"in ([\d.]+) wall s", float),
            "real_time_factor": grab(r"real-time factor ([\d.]+)", float),
            "odom_messages": grab(r"factor [\d.]+\), (\d+) odom messages", int),
            "distance_m": grab(r"Total distance calculated: ([\d.]+) m", float),
            "pytest_summary": grab(r"=+ ([^=\n]*(?:passed|failed|error)[^=\n]*) =+"),
            "sim_pixi_env": grab(r"Launching simulator with pixi env: (\S+)")}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("task", help="pixi task to benchmark, e.g. test-rerun-recording")
    ap.add_argument("--label", help="name for this machine/run in the summary")
    ap.add_argument("--json", dest="json_path", help="where to write the JSON result")
    args = ap.parse_args()

    out_dir = REPO / "test_outputs"
    bench_dir = out_dir / f"bench_{args.task}_{time.strftime('%Y%m%d-%H%M%S')}"
    bench_dir.mkdir(parents=True, exist_ok=True)
    rc, wall, text, proc_samples, gpu_samples = run_task(args.task, bench_dir)

    r = {"label": args.label, "task": args.task, "exit_code": rc, "task_wall_sec": round(wall, 1),
         "bench_dir": str(bench_dir), "host": host_info(), **parse_task_output(text)}
    sim_logs = sorted(bench_dir.glob("pytest/**/sim.log"), key=lambda p: p.stat().st_mtime)
    r["sim_log"] = str(sim_logs[-1]) if sim_logs else None
    r["control_log"] = str(next(iter(sorted(bench_dir.glob("pytest/**/control.log"))), "")) or None
    r.update(parse_sim_log(sim_logs[-1]) if sim_logs else {})
    r["processes"] = summarize(proc_samples, ("samples", "cpu_mean", "cpu_peak", "rss_peak_mb"))
    r["gpu"] = summarize(gpu_samples, ("samples", "util_mean", "util_peak", "mem_peak_mb"))

    rows = [("exit code", rc), ("test result", r["pytest_summary"]), ("total task wall s", r["task_wall_sec"]),
            ("simulator ready s", r["sim_ready_sec"]), ("warm-up s", r.get("warmup_sec")),
            ("sim-time span s", r["sim_span_sec"]), ("wall span s", r["wall_span_sec"]),
            ("**real-time factor**", r["real_time_factor"]), ("odom messages", r["odom_messages"]),
            ("distance m", r["distance_m"]), ("pixi sim env", r["sim_pixi_env"]),
            ("warp", r.get("warp")), ("warp devices", "; ".join(r.get("warp_devices") or []))]
    for name, s in r["processes"].items():
        rows += [(f"{name} CPU% mean/peak", f"{s['cpu_mean']:.0f} / {s['cpu_peak']:.0f}"),
                 (f"{name} RSS peak MB", f"{s['rss_peak_mb']:.0f}")]
    for name, s in r["gpu"].items():
        rows += [(f"GPU {name} util% mean/peak", f"{s['util_mean']:.0f} / {s['util_peak']:.0f}"),
                 (f"GPU {name} mem peak MB", f"{s['mem_peak_mb']:.0f}")]

    h = r["host"]
    print(f"\n## {args.label or h['hostname']} - `{args.task}` @ {h['git_sha']}")
    print(f"{h['platform']} | {h['cpu']} | {h['cpu_count']} cpus | {h['ram_gb']} GB | py {h['python']}\n")
    print("| metric | value |\n| --- | --- |")
    for k, v in rows:
        print(f"| {k} | {'' if v is None else v} |")
    print("\nThe Newton loop is real-time paced, so RTF <= 1.0 by design; < 1.0 means the machine "
          "could not keep up.")

    json_path = Path(args.json_path or out_dir / f"bench_{args.task}_{args.label or h['hostname']}.json")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(r, indent=2, default=str))
    print(f"\nJSON: {json_path}\nLogs: {bench_dir}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
