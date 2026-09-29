#!/usr/bin/env python3
"""Turn simulation CSVs into PNG curves and a self-contained HTML report."""
from __future__ import annotations

import argparse
import base64
import csv
from datetime import datetime, timezone
import html
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = {
    "step": ("阶跃响应", "Step response", "目标转速从零升至设定值，观察跟踪过程。"),
    "load": ("负载扰动", "Load disturbance", "施加并移除等效负载，观察转速误差与控制输出的变化。"),
    "saturation": ("输出饱和", "Output saturation", "先设定较高目标，再降低目标，观察输出受限及恢复过程。"),
}
NUMBER_COLUMNS = (
    "time_s", "setpoint_rpm", "speed_rpm", "error_rpm", "pwm", "load_rpm",
    "pid_updated", "kp", "ki", "kd", "sample_time_ms",
)


def load_csv(path: Path, scenario: str) -> list[dict[str, float]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = {"scenario", *NUMBER_COLUMNS} - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path.name}: missing CSV columns: {', '.join(sorted(missing))}")
        rows = []
        for line, raw in enumerate(reader, start=2):
            if raw["scenario"] != scenario:
                raise ValueError(f"{path.name}:{line}: unexpected scenario {raw['scenario']!r}")
            row = {name: float(raw[name]) for name in NUMBER_COLUMNS}
            if not all(math.isfinite(value) for value in row.values()):
                raise ValueError(f"{path.name}:{line}: non-finite numeric value")
            if rows and row["time_s"] <= rows[-1]["time_s"]:
                raise ValueError(f"{path.name}:{line}: timestamps must increase")
            if abs(row["error_rpm"] - (row["setpoint_rpm"] - row["speed_rpm"])) > 0.001:
                raise ValueError(f"{path.name}:{line}: error_rpm does not match target minus speed")
            if row["sample_time_ms"] <= 0 or row["pid_updated"] not in (0, 1):
                raise ValueError(f"{path.name}:{line}: invalid PID timing metadata")
            if rows and any(row[key] != rows[0][key] for key in ("kp", "ki", "kd", "sample_time_ms")):
                raise ValueError(f"{path.name}:{line}: PID settings changed within a scenario")
            rows.append(row)
    if len(rows) < 2:
        raise ValueError(f"{path.name}: at least two data samples are required")
    return rows


def integral(rows: list[dict[str, float]], squared: bool = False) -> float:
    total = 0.0
    for left, right in zip(rows, rows[1:]):
        a, b = left["error_rpm"], right["error_rpm"]
        a, b = (a * a, b * b) if squared else (abs(a), abs(b))
        total += 0.5 * (a + b) * (right["time_s"] - left["time_s"])
    return total


def metrics(rows: list[dict[str, float]]) -> dict[str, float]:
    duration = rows[-1]["time_s"] - rows[0]["time_s"]
    iae = integral(rows)
    return {
        "duration": duration,
        "mae": iae / duration,
        "rmse": math.sqrt(integral(rows, squared=True) / duration),
        "iae": iae,
        "peak": max(abs(row["error_rpm"]) for row in rows),
        "final": abs(rows[-1]["error_rpm"]),
    }


def plot_csv(rows: list[dict[str, float]], title: str, destination: Path, plt) -> None:
    times = [row["time_s"] for row in rows]
    values = lambda key: [row[key] for row in rows]
    fig, axes = plt.subplots(3, 1, figsize=(11.5, 8), sharex=True, constrained_layout=True)
    first = rows[0]
    fig.suptitle(
        f"{title} | Software simulation\n"
        f"Kp={first['kp']:g}, Ki={first['ki']:g}, Kd={first['kd']:g} | "
        f"PID sample time: {first['sample_time_ms']:g} ms", fontsize=14,
    )
    axes[0].plot(times, values("setpoint_rpm"), color="#667085", ls="--", lw=1.8, label="Setpoint")
    axes[0].plot(times, values("speed_rpm"), color="#167d8d", lw=1.8, label="Simulated speed")
    axes[0].set_ylabel("Speed (rpm)")
    axes[1].step(times, values("pwm"), where="post", color="#9459c3", lw=1.7, label="PWM command")
    axes[1].set_ylabel("PWM command")
    axes[2].plot(times, values("error_rpm"), color="#d46b39", lw=1.6, label="Tracking error")
    axes[2].plot(times, values("load_rpm"), color="#667085", ls=":", lw=1.5, label="Equivalent load")
    axes[2].set_ylabel("Error / load (rpm)")
    axes[2].set_xlabel("Time (s)")
    for axis in axes:
        axis.grid(True, color="#d8dee5", alpha=0.7, linewidth=0.6)
        axis.legend(loc="best", frameon=False, fontsize=9)
        axis.spines[["top", "right"]].set_visible(False)
        axis.set_xlim(times[0], times[-1])
    fig.savefig(destination, dpi=160, facecolor="white")
    plt.close(fig)


def plot_overview(all_rows: dict[str, list[dict[str, float]]], destination: Path, plt) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11.5, 9), constrained_layout=True)
    fig.suptitle("Motor PID Control | Software Simulation", fontsize=17, fontweight="bold")
    for axis, (scenario, rows) in zip(axes, all_rows.items()):
        settings = rows[0]
        times = [row["time_s"] for row in rows]
        axis.plot(times, [row["setpoint_rpm"] for row in rows], color="#667085", ls="--", lw=1.6, label="Setpoint")
        axis.plot(times, [row["speed_rpm"] for row in rows], color="#167d8d", lw=1.8, label="Simulated speed")
        axis.set_title(
            f"{SCENARIOS[scenario][1]}  |  Kp={settings['kp']:g}, "
            f"Ki={settings['ki']:g}, Kd={settings['kd']:g}  |  "
            f"PID period={settings['sample_time_ms']:g} ms", fontsize=11, loc="left",
        )
        axis.set_ylabel("Speed (rpm)")
        axis.set_xlabel("Time (s)")
        axis.set_xlim(times[0], times[-1])
        axis.grid(True, color="#d8dee5", alpha=0.7, linewidth=0.6)
        axis.legend(loc="best", frameon=False, fontsize=9)
        axis.spines[["top", "right"]].set_visible(False)
    fig.savefig(destination, dpi=160, facecolor="white")
    plt.close(fig)


def build_report(results_dir: Path, plt) -> Path:
    summary_rows, sections = [], []
    all_rows = {}
    for scenario, (name, english, description) in SCENARIOS.items():
        source = results_dir / f"{scenario}.csv"
        rows = load_csv(source, scenario)
        all_rows[scenario] = rows
        stats, settings = metrics(rows), rows[0]
        plot_path = results_dir / f"{scenario}.png"
        plot_csv(rows, english, plot_path, plt)
        encoded = base64.b64encode(plot_path.read_bytes()).decode("ascii")
        image_url = f"data:image/png;base64,{encoded}"
        summary_rows.append(
            f'<tr><th><a href="#{scenario}">{name}</a></th>'
            f'<td>{stats["mae"]:.2f}</td><td>{stats["rmse"]:.2f}</td>'
            f'<td>{stats["iae"]:.2f}</td><td>{stats["peak"]:.2f}</td><td>{stats["final"]:.2f}</td></tr>'
        )
        settings_text = (
            f'Kp {settings["kp"]:g} · Ki {settings["ki"]:g} · Kd {settings["kd"]:g} · '
            f'PID 采样周期 {settings["sample_time_ms"]:g} ms · '
            f'记录时长 {stats["duration"]:g} s · {len(rows)} 条记录'
        )
        sections.append(f'''
        <section id="{scenario}" class="scenario">
          <div class="section-top"><h2>{name}</h2><a href="#overview">返回指标表 ↑</a></div>
          <p>{description}</p><p class="settings">{html.escape(settings_text)}</p>
          <a href="{image_url}" download="{scenario}.png" title="点击下载曲线 PNG">
            <img src="{image_url}" alt="{english} simulation curves" loading="lazy">
          </a>
          <p class="caption">点击曲线可保存 PNG。数据来源：{html.escape(source.name)}。</p>
        </section>''')
    plot_overview(all_rows, results_dir / "overview.png", plt)
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    document = f'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>嵌入式电机 PID 控制 纯软件仿真报告</title>
<style>
:root{{color-scheme:light;font-family:"Segoe UI","Microsoft YaHei",sans-serif;color:#1f2937;background:#eef2f5}}
*{{box-sizing:border-box}}body{{margin:0}}main{{max-width:1080px;margin:auto;padding:44px 28px 60px;background:white;min-height:100vh}}
.eyebrow{{color:#167d8d;font-size:13px;font-weight:700;letter-spacing:.05em}}h1{{font-size:32px;line-height:1.35;margin:14px 0 18px}}
h2{{font-size:23px;margin:0}}p{{line-height:1.8}}.lead{{max-width:780px;color:#475467}}a{{color:#126878;text-underline-offset:3px}}
nav{{display:flex;flex-wrap:wrap;gap:12px;margin:24px 0 34px}}nav a{{padding:8px 14px;background:#e8f2f3;border-radius:6px;text-decoration:none}}
.table-scroll{{overflow-x:auto;margin-top:18px}}table{{width:100%;border-collapse:collapse;white-space:nowrap;font-size:14px}}
th,td{{padding:14px 12px;border-bottom:1px solid #e0e5ea;text-align:right}}th:first-child{{text-align:left}}thead{{background:#f1f5f7}}
.note,.caption{{color:#667085;font-size:13px}}.scenario{{margin-top:46px;padding-top:26px;border-top:1px solid #d9e0e6;scroll-margin-top:22px}}
.section-top{{display:flex;align-items:baseline;justify-content:space-between;gap:16px}}.section-top a{{font-size:13px}}
.settings{{font-size:14px;color:#344054;background:#f5f7f9;padding:11px 14px;border-radius:5px}}img{{width:100%;height:auto;display:block}}
footer{{margin-top:38px;border-top:1px solid #d9e0e6;padding-top:20px;color:#667085;font-size:12px}}
@media(max-width:600px){{main{{padding:26px 16px 40px}}h1{{font-size:25px}}th,td{{padding:11px 10px}}.section-top{{align-items:center}}}}
@media print{{body{{background:white}}main{{padding:0}}nav,.section-top a{{display:none}}.scenario{{break-inside:avoid}}}}
</style></head><body><main>
<div class="eyebrow">纯软件仿真 · 非硬件实测</div>
<h1>嵌入式电机 PID 控制仿真报告</h1>
<p class="lead">使用 PID 控制器和理想电机模型，比较阶跃跟踪、负载扰动与输出饱和三个场景。
参数和误差指标均来自本次 CSV 数据；曲线展示的是模型行为。</p>
<nav aria-label="场景导航"><a href="#overview">误差指标</a><a href="#step">阶跃响应</a><a href="#load">负载扰动</a><a href="#saturation">输出饱和</a></nav>
<section id="overview"><h2>本次运行的误差指标</h2><div class="table-scroll"><table>
<thead><tr><th>场景</th><th>MAE<br>rpm</th><th>RMSE<br>rpm</th><th>IAE<br>rpm·s</th><th>最大绝对误差<br>rpm</th><th>末点绝对误差<br>rpm</th></tr></thead>
<tbody>{''.join(summary_rows)}</tbody></table></div>
<p class="note">误差 e = 目标转速 − 仿真转速。全程 IAE = ∫|e|dt，MAE = IAE/T，RMSE = √(∫e²dt/T)。
积分按 CSV 时间戳使用梯形法计算，T 为首末记录时间差；包含启动、目标切换与负载变化。
末点误差不代表稳态误差，输出饱和场景的不可达目标也计入指标。</p></section>
{''.join(sections)}
<footer>生成时间：{stamp}。图像已嵌入本 HTML，可离线打开或单独分享。PNG 曲线另存于同目录；CSV 是本报告的原始数据。</footer>
</main></body></html>'''
    destination = results_dir / "report.html"
    destination.write_text(document, encoding="utf-8")
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "Plotting requires matplotlib. Install it explicitly, then rerun:\n"
            f'  "{sys.executable}" -m pip install -r "{ROOT / "requirements-demo.txt"}"\n'
            "To generate only CSV data, use: python lab/run_demo.py --skip-report"
        ) from exc
    print("Report written: " + str(build_report(args.results_dir.expanduser().resolve(), plt)))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"Report failed: {exc}", file=sys.stderr)
        sys.exit(1)
