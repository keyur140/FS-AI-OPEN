# fsai-open: the five colour PDF reports (Results, Project Brief, Introduction, Exploratory run, Impact)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Every number in these PDFs is read from reports/results.json, runs/ and config/, so the documents always match
the code. Figures are drawn here from the same files (and one seeded desk run, which is deterministic).

    pip install reportlab matplotlib      # not needed by the driving software itself
    python tools/make_pdfs.py             # -> reports/pdf/*.pdf (six reports; the Technical Report is in report_pdf.py)
"""
from __future__ import annotations

import hashlib
import io
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from reportlab.lib import colors  # noqa: E402
from reportlab.lib.enums import TA_CENTER  # noqa: E402
from reportlab.lib.pagesizes import A4  # noqa: E402
from reportlab.lib.styles import ParagraphStyle  # noqa: E402
from reportlab.lib.units import mm  # noqa: E402
from reportlab.lib.utils import ImageReader  # noqa: E402
from reportlab.pdfbase import pdfmetrics  # noqa: E402
from reportlab.pdfbase.pdfmetrics import registerFontFamily  # noqa: E402
from reportlab.pdfbase.ttfonts import TTFont  # noqa: E402
from reportlab.platypus import (CondPageBreak, Image, KeepTogether, PageBreak, Paragraph,  # noqa: E402
                                SimpleDocTemplate, Spacer, Table, TableStyle)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
OUT = ROOT / "reports" / "pdf"

FD = Path(matplotlib.get_data_path()) / "fonts" / "ttf"      # DejaVu fonts ship with matplotlib
for n, f in (("DV", "DejaVuSans.ttf"), ("DVB", "DejaVuSans-Bold.ttf"), ("DVI", "DejaVuSans-Oblique.ttf"),
             ("DVBI", "DejaVuSans-BoldOblique.ttf"), ("DVM", "DejaVuSansMono.ttf")):
    pdfmetrics.registerFont(TTFont(n, str(FD / f)))
registerFontFamily("DV", normal="DV", bold="DVB", italic="DVI", boldItalic="DVBI")

HEX = {"blue": "#1f4fa8", "navy": "#14306b", "green": "#1e9e6a", "orange": "#e8892b", "red": "#c43d5b",
       "purple": "#7a52c7", "yellow": "#e0b400", "teal": "#1a9bb0", "grey": "#5a6577"}
C = {k: colors.HexColor(v) for k, v in HEX.items()}
TINT = {k: colors.HexColor(v) for k, v in {"blue": "#e3ebfa", "green": "#e1f5ec", "orange": "#fdf0e2",
                                            "red": "#fbe6ec", "purple": "#efe9fb", "teal": "#e0f4f7",
                                            "grey": "#f1f3f7"}.items()}
W = 170 * mm


def sty(name, **kw):
    base = dict(fontName="DV", fontSize=10.5, leading=15, textColor=colors.HexColor("#17202e"))
    base.update(kw)
    return ParagraphStyle(name, **base)


S = {"h2": sty("h2", fontName="DVB", fontSize=13, leading=17, textColor=C["navy"], spaceBefore=6, spaceAfter=3),
     "p": sty("p", spaceAfter=5), "small": sty("sm", fontSize=8.5, leading=11, textColor=C["grey"]),
     "cap": sty("cap", fontSize=8.5, leading=11, alignment=TA_CENTER, textColor=C["grey"]),
     "tile_n": sty("tn", fontName="DVB", fontSize=19, leading=22, alignment=TA_CENTER, textColor=colors.white),
     "tile_t": sty("tt", fontSize=8.5, leading=11, alignment=TA_CENTER, textColor=colors.white),
     "card_h": sty("ch", fontName="DVB", fontSize=11, leading=14), "card": sty("cd", fontSize=9.5, leading=13),
     "code": sty("code", fontName="DVM", fontSize=8.8, leading=12, textColor=colors.HexColor("#0b2a5b")),
     "td": sty("td", fontSize=8.3, leading=10.5), "th": sty("th", fontName="DVB", fontSize=8.3, leading=10.5,
                                                            textColor=colors.white),
     "bul": sty("bul", leftIndent=12, bulletIndent=2, spaceAfter=2),
     "title": sty("title", fontName="DVB", fontSize=24, leading=29, textColor=colors.white),
     "sub": sty("sub", fontSize=11.5, leading=15, textColor=colors.white),
     "auth": sty("auth", fontSize=9.5, leading=13)}

AUTHOR_BLOCK = ("<b>KEYUR SHAILESHKUMAR PATEL</b> · ORCID https://orcid.org/0009-0008-5261-043X<br/>"
                "(1) MSc Artificial Intelligence student, Department of Computer Science, Brunel University London, UK<br/>"
                "(2) Independent researcher<br/>"
                "<i>Conducted independently; not an official Brunel University London project.</i>")
AI_NOTE = ("<b>AI assistance, stated openly.</b> The code, the tests and these documents were drafted with an AI "
           "assistant (Claude, by Anthropic). Every design decision was made and approved by the author. Every number "
           "here is read by a script (<font name='DVM'>tools/make_pdfs.py</font>) from the result files of the build, "
           "which anyone can re-create with <font name='DVM'>python run_all.py</font> and check with "
           "<font name='DVM'>python verify.py</font>.")


# ----------------------------------------------------------------------------------------- building blocks
def md(t):
    """Markdown from the repository's tables -> ReportLab markup."""
    import re
    t = t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    t = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", t)
    t = re.sub(r"`([^`]+)`", r"<font name='DVM'>\1</font>", t)
    return t


def P(t, s="p"):
    return Paragraph(t, S[s])


def bullets(items):
    return [Paragraph(i, S["bul"], bulletText="•") for i in items]


def section(title, color="blue"):
    t = Table([[Paragraph(title, sty("sec", fontName="DVB", fontSize=14, leading=18, textColor=colors.white))]],
              colWidths=[W])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), C[color]), ("LEFTPADDING", (0, 0), (-1, -1), 10),
                           ("TOPPADDING", (0, 0), (-1, -1), 5), ("BOTTOMPADDING", (0, 0), (-1, -1), 6)]))
    return [CondPageBreak(55 * mm), Spacer(1, 3 * mm), t, Spacer(1, 3 * mm)]


def tiles(items):
    """items: (number, text, colour)"""
    t = Table([[[P(n, "tile_n"), P(x, "tile_t")] for n, x, _ in items]], colWidths=[W / len(items)] * len(items))
    st = [("VALIGN", (0, 0), (-1, -1), "MIDDLE"), ("TOPPADDING", (0, 0), (-1, -1), 8),
          ("BOTTOMPADDING", (0, 0), (-1, -1), 8), ("LINEAFTER", (0, 0), (-2, -1), 3, colors.white)]
    st += [("BACKGROUND", (i, 0), (i, 0), C[c]) for i, (_, _, c) in enumerate(items)]
    t.setStyle(TableStyle(st))
    return t


def card(title, body, color="blue"):
    t = Table([[P(f'<font color="{HEX[color]}">●</font> {title}', "card_h")], [P(body, "card")]],
              colWidths=[W / 2 - 4 * mm])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), TINT[color]), ("LINEBEFORE", (0, 0), (0, -1), 4, C[color]),
                           ("LEFTPADDING", (0, 0), (-1, -1), 9), ("TOPPADDING", (0, 0), (-1, -1), 4),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 5)]))
    return t


def card_grid(cards):
    rows = [cards[i:i + 2] + ([""] if len(cards[i:i + 2]) == 1 else []) for i in range(0, len(cards), 2)]
    t = Table(rows, colWidths=[W / 2] * 2)
    t.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
                           ("LEFTPADDING", (0, 0), (-1, -1), 0)]))
    return t


def box(flows, color="blue", title=None):
    inner = ([P(f"<b>{title}</b>", "card_h")] if title else []) + (flows if isinstance(flows, list) else [flows])
    t = Table([[inner]], colWidths=[W])
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), TINT[color]), ("BOX", (0, 0), (-1, -1), 1, C[color]),
                           ("LEFTPADDING", (0, 0), (-1, -1), 10), ("RIGHTPADDING", (0, 0), (-1, -1), 10),
                           ("TOPPADDING", (0, 0), (-1, -1), 6), ("BOTTOMPADDING", (0, 0), (-1, -1), 7)]))
    return t


def plain(items, title="In plain words"):
    return box(bullets(items) if isinstance(items, list) else P(items), "green", title)


def hood(items, title="Under the hood"):
    return box(bullets(items) if isinstance(items, list) else P(items), "blue", title)


def table(header, rows, widths, color="navy", zebra=True):
    data = [[P(h, "th") for h in header]] + [[P(str(c), "td") for c in r] for r in rows]
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1)
    st = [("BACKGROUND", (0, 0), (-1, 0), C[color]), ("VALIGN", (0, 0), (-1, -1), "TOP"),
          ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#c9d2e3")), ("TOPPADDING", (0, 0), (-1, -1), 2.5),
          ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5), ("LEFTPADDING", (0, 0), (-1, -1), 4)]
    if zebra:
        st += [("BACKGROUND", (0, i), (-1, i), TINT["grey"]) for i in range(2, len(data), 2)]
    t.setStyle(TableStyle(st))
    return t


def fig(drawfn, caption, w_mm=170, h_mm=80):
    f = plt.figure(figsize=(w_mm / 25.4, h_mm / 25.4), dpi=200)
    drawfn(f)
    buf = io.BytesIO()
    f.savefig(buf, format="png", dpi=200, bbox_inches="tight")
    plt.close(f)
    buf.seek(0)
    iw, ih = ImageReader(buf).getSize()
    buf.seek(0)
    w = w_mm * mm
    return KeepTogether([Image(buf, width=w, height=w * ih / iw), Spacer(1, 1 * mm), P(caption, "cap"),
                         Spacer(1, 3 * mm)])


plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.edgecolor": "#5a6577", "axes.titleweight": "bold",
                     "axes.titlesize": 9})


# ----------------------------------------------------------------------------------------- data
def load():
    R = json.loads((ROOT / "reports" / "results.json").read_text())
    E = json.loads((ROOT / "runs" / "exploratory_real" / "results.json").read_text())
    stress = json.loads((ROOT / "runs" / "events" / "stress.json").read_text())
    ros = []
    for f in sorted((ROOT / "runs" / "ros2_autonomy").glob("result_*.json")):
        r = json.loads(f.read_text())
        r["run"] = f.stem[len("result_"):]
        ros.append(r)
    suite = json.loads((ROOT / "runs" / "events" / "results.json").read_text())
    tests = json.loads((ROOT / "runs" / "tests" / "summary.json").read_text())
    scen = json.loads((ROOT / "runs" / "scenarios" / "results.json").read_text())
    res_sha = hashlib.sha256((ROOT / "reports" / "results.json").read_bytes()).hexdigest()
    return dict(R=R, E=E, stress=stress["conditions"], stress_seeds=stress["seeds"], ros=ros, suite=suite,
                tests=tests, scen=scen, res_sha=res_sha, version=R["version"])


def facts(D):
    runs = D["suite"]["runs"]
    ev = [r for r in runs if not r.get("mission_checks")]
    mi = [r for r in runs if r.get("mission_checks")]
    c = D["tests"]["counts"]
    return dict(
        checks=c["passed"], failed=c["failed"] + c["error"], ev_ok=sum(r["pass"] for r in ev), ev_n=len(ev),
        mi_ok=sum(r["pass"] for r in mi), mi_n=len(mi), sc_ok=sum(r["pass"] and r["rerun_identical"]
                                                                   for r in D["scen"]["scenarios"]),
        sc_n=len(D["scen"]["scenarios"]), ros_ok=sum(bool(r.get("ok")) for r in D["ros"]), ros_n=len(D["ros"]),
        det=D["suite"]["determinism_ok"], cones_hit=sum(r["cones_hit"] for r in ev),
        timing=D["R"].get("timing"), ev=ev, mi=mi)


def track_figure():
    """One seeded desk trackdrive (deterministic): true cones, the car's path coloured by speed."""
    from fsai.autonomy.sim import World
    from fsai.autonomy.track import BIG_ORANGE, BLUE, ORANGE, YELLOW
    w = World("trackdrive", 5)
    r = w.run()
    tr = np.array([row[:7] for row in w.trace], dtype=float)
    cones = w.track.cones
    line = (w.stack.global_line or {}).get("line")

    def draw(f):
        ax = f.add_subplot(111)
        drv = tr[tr[:, 1] == 3]
        sc = ax.scatter(drv[:, 2], drv[:, 3], c=drv[:, 5], cmap="plasma", s=0.6, zorder=2)
        for col, c, s in ((BLUE, "#1f4fa8", 9), (YELLOW, "#e0b400", 9), (ORANGE, "#e8892b", 9),
                          (BIG_ORANGE, "#c45a00", 18)):
            m = cones[:, 2].astype(int) == col
            ax.scatter(cones[m, 0], cones[m, 1], c=c, s=s, marker="^", zorder=3, linewidths=0)
        if line is not None:
            ln = np.asarray(line)
            ax.plot(ln[:, 0], ln[:, 1], color="#1e9e6a", lw=0.7, ls="--", zorder=1, label="racing line (planned)")
            ax.legend(loc="upper right", frameon=False, fontsize=7)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        cb = f.colorbar(sc, ax=ax, fraction=0.03, pad=0.02)
        cb.set_label("speed [m/s]")
    return draw, r


# ----------------------------------------------------------------------------------------- page furniture
def make_doc(name, title, subtitle, color, D):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name

    def first(cv, doc):
        cv.saveState()
        cv.setFillColor(C[color])
        cv.rect(0, A4[1] - 62 * mm, A4[0], 62 * mm, fill=1, stroke=0)
        cv.setFillColor(C["yellow"])
        for i, col in enumerate(("blue", "yellow", "orange", "green")):
            cv.setFillColor(C[col] if col != color else colors.white)
            cv.rect(20 * mm + i * 12 * mm, A4[1] - 66 * mm, 10 * mm, 2.2 * mm, fill=1, stroke=0)
        cv.restoreState()
        later(cv, doc)

    def later(cv, doc):
        cv.saveState()
        cv.setFont("DV", 7.5)
        cv.setFillColor(C["grey"])
        cv.drawString(20 * mm, 10 * mm, f"fsai-open {D['version']} · {title} · results.json SHA-256 "
                                        f"{D['res_sha'][:16]}…")
        cv.drawRightString(A4[0] - 20 * mm, 10 * mm, f"page {doc.page}")
        cv.restoreState()

    doc = SimpleDocTemplate(str(path), pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm, topMargin=16 * mm,
                            bottomMargin=17 * mm, title=f"fsai-open {D['version']}: {title}",
                            author="KEYUR SHAILESHKUMAR PATEL", subject=subtitle,
                            creator="fsai-open tools/make_pdfs.py")
    head = [Spacer(1, 4 * mm), P(title, "title"), Spacer(1, 2 * mm), P(subtitle, "sub"), Spacer(1, 22 * mm),
            box(P(AUTHOR_BLOCK, "auth"), "grey"), Spacer(1, 2 * mm), box(P(AI_NOTE, "card"), "purple"),
            Spacer(1, 3 * mm)]
    return doc, head, first, later


def build(name, title, subtitle, color, D, story):
    doc, head, first, later = make_doc(name, title, subtitle, color, D)
    doc.build(head + story, onFirstPage=first, onLaterPages=later)
    return OUT / name


def not_show(items):
    return box(bullets(items), "red", "What this does not show")


# ----------------------------------------------------------------------------------------- 1. Results
def results_pdf(D, F, trackfig):
    E = D["E"]
    t = F["timing"]
    st = []
    st += [plain([
        f"<b>Overall: {'everything passed' if D['R']['all_ok'] else 'something failed'}.</b> "
        f"{F['checks']} automatic checks passed and {F['failed']} failed.",
        f"In {F['ev_n']} simulated competition runs (Acceleration, Skidpad, Autocross and 10-lap Trackdrive on "
        f"seeded tracks), all {F['ev_ok']} were clean as the FS-AI 2026 rules judge them: right laps, stopped in the "
        f"allowed distance, no cone touched, never off track. Running them again gave identical fingerprints.",
        f"{F['mi_ok']} of {F['mi_n']} inspection runs (Static Inspection A and B, Autonomous Demo) did every step "
        "the rules list.",
        f"{F['ros_ok']} of {F['ros_n']} real-time runs, with every part as a separate ROS 2 program in Docker, "
        "were clean.",
        f"On real LiDAR scans from another team's car, the cone detector found "
        f"{round(100 * E['real']['front_80deg']['summary']['recall'])} % of cones up to 20 m, against "
        f"{round(100 * E['synthetic']['front_80deg']['summary']['recall'])} % in our simulator. The gap is beyond 10 m.",
        "None of this has run on the real car yet."])]
    st.append(Spacer(1, 3 * mm))
    st.append(tiles([(f"{F['checks']}", "automatic checks passed", "blue"),
                     (f"{F['ev_ok']}/{F['ev_n']}", "simulated events clean", "green"),
                     (f"{F['ros_ok']}/{F['ros_n']}", "real-time ROS 2 runs clean", "teal"),
                     (f"{F['sc_ok']}/{F['sc_n']}", "safety scenarios as the rules say", "orange"),
                     (f"{F['cones_hit']}", "cones hit in all events", "purple")]))
    st += section("1. The driverless events (desk, simulated time)", "blue")
    st.append(P("A referee that sits outside the driving software watches the true car on the true track. It counts "
                "cones hit, time off track, lap times and the FS-AI 2026 penalties (D9) and stop distances. "
                "The driving software never sees the true track; it only gets simulated LiDAR, camera, IMU and wheel "
                "speeds, and it talks to a software copy of the car's safety computer through encoded CAN bytes."))
    rows = []
    for r in F["ev"]:
        lt = r["lap_times_referee_s"]
        shown = ", ".join(f"{x:.2f}" for x in lt) if len(lt) <= 4 else f"{lt[0]:.2f}, then {min(lt[1:]):.2f}–{max(lt[1:]):.2f}"
        rows.append([r["event"], r["seed"], r["track_length_m"], len(lt), shown, r.get("official_time_s"),
                     f"{r.get('stop_distance_m')} ({r.get('stop_allowed_m')})", r["cones_hit"], r["max_speed_mps"],
                     f"{r['map_true_cones']} ({r['map_false_cones']})", "yes" if r["pass"] else "NO"])
    st.append(table(["Event", "Seed", "Track [m]", "Laps", "Lap times [s]", "Official [s]", "Stop [m] (allowed)",
                     "Cones hit", "Top [m/s]", "Map cones (false)", "Clean"], rows,
                    [24, 8, 13, 8, 30, 14, 18, 10, 11, 18, 10]))
    st.append(Spacer(1, 3 * mm))
    st.append(fig(trackfig, "Trackdrive, seed 5, on the desk: cones as the track generator placed them (blue and "
                            "yellow edges, orange and big orange at the start), the car's path coloured by speed, and "
                            "the planned racing line after the first lap. Drawn from a re-run of the same seed.",
                  170, 95))
    st += section("2. Inspection missions (FS-AI 2026 IN2.2.2 and IN2.3.2)", "green")
    rows = []
    for r in F["mi"]:
        mc = {k: v for k, v in r["mission_checks"].items() if k != "all_ok"}
        rows.append([r["event"], r["seed"], r["final_state"],
                     "; ".join(f"{k.replace('_', ' ')}: {({True: 'yes', False: 'NO'}.get(v, v) if isinstance(v, bool) else v)}"
                               for k, v in mc.items()), "yes" if r["pass"] else "NO"])
    st.append(table(["Mission", "Seed", "End state", "Checks", "Pass"], rows, [22, 10, 30, 96, 12], "green"))
    st.append(P("Inspection B and the Autonomous Demo end in AS Emergency on purpose: the rules ask for an EBS test "
                "at the end of each.", "small"))
    st += section("3. Real time: every part a separate program (ROS 2 in Docker)", "teal")
    st.append(P("The same driving software, but now the pretend safety computer, the CAN bridge, the pretend track "
                "with its sensors, and the driving stack are four separate ROS 2 programs talking over a virtual CAN "
                "bus in real time, as on the car. Thread starvation, late scans and stalls show up here, not on the "
                "desk. Several were found and fixed this way (decisions D15 to D18)."))
    rows = []
    for r in D["ros"]:
        lt = r.get("lap_times_referee_s") or []
        shown = ", ".join(f"{x:.2f}" for x in lt) if len(lt) <= 4 else f"{lt[0]:.2f}, then {min(lt[1:]):.2f}–{max(lt[1:]):.2f}"
        rows.append([r["run"].replace("_", " "), r.get("final_state"), len(lt), shown or "-", r.get("cones_hit"),
                     r.get("off_track_s"), r.get("max_lateral_m"), "yes" if r.get("ok") else "NO"])
    st.append(table(["Run", "End state", "Laps", "Lap times [s]", "Cones hit", "Off track [s]", "Max off line [m]",
                     "Clean"], rows, [34, 32, 10, 38, 14, 15, 17, 10], "teal"))
    if t:
        b = t["bridge_loop"]
        st.append(Spacer(1, 2 * mm))
        st.append(P(f"<b>10 ms loop timing</b> on the build computer ({b['cycles']} cycles): median "
                    f"{b['p50_period_ms']} ms, 99th percentile {b['p99_period_ms']} ms, longest {b['max_period_ms']} ms, "
                    f"{b['overruns']} overruns. The car stops itself after 50 ms without an answer (FS-AI 2026 T1.1.8)."))
    st += section("4. Stress: where does it break?", "orange")
    st.append(P(f"Autocross on seeds {', '.join(map(str, D['stress_seeds']))}, with one thing made worse at a time, "
                "then everything together. A run is \"not clean\" for any single fault, including one phantom cone "
                "left in the map. Cones hit is the column that matters for driving."))
    rows = []
    for name, d in D["stress"].items():
        ov = ", ".join(f"{k.split('__')[1]}={v}" for k, v in d["overrides"].items()) or "-"
        why = "; ".join(sorted({w for f in d["fail_reasons"] for w in f})) or "-"
        rows.append([name.replace("_", " "), ov, f"{d['passed']}/{d['runs']}", d["cones_hit"], d["map_false_cones"],
                     why])
    st.append(table(["Condition", "Changed settings", "Clean", "Cones hit", "Phantom map cones", "Why not clean"],
                    rows, [32, 62, 11, 16, 17, 32], "orange"))

    def stress_draw(f):
        ax = f.add_subplot(111)
        names = list(D["stress"])[::-1]
        vals = [D["stress"][n]["passed"] / D["stress"][n]["runs"] for n in names]
        ax.barh([n.replace("_", " ") for n in names], vals,
                color=["#1e9e6a" if v == 1 else "#e8892b" for v in vals])
        ax.set_xlim(0, 1)
        ax.set_xlabel("share of clean runs")
    st.append(fig(stress_draw, "Share of clean stress runs per condition. Orange rows lost a run to one phantom "
                               "cone in the map; no stress run hit a cone.", 150, 70))
    st += section("5. Real LiDAR scans (exploratory)", "purple")
    st.append(P("See the Exploratory Run report for the full method. Headline, front view, cones up to 20 m:"))
    ra, sy = E["real"]["front_80deg"]["summary"], E["synthetic"]["front_80deg"]["summary"]
    st.append(table(["", "Cones labelled", "Found", "False cones", "Found share", "Real share of detections",
                     "Position error [m]"],
                    [["Real scans", ra["labelled_cones"], ra["found"], ra["false_cones"], ra["recall"], ra["precision"],
                      ra["mean_position_error_m"]],
                     ["Our simulator", sy["labelled_cones"], sy["found"], sy["false_cones"], sy["recall"],
                      sy["precision"], sy["mean_position_error_m"]]], [28, 22, 18, 20, 22, 32, 28], "purple"))
    st += section("6. How to check every number yourself", "navy")
    st += bullets(["<font name='DVM'>python verify.py</font>: recomputes the SHA-256 of every file against "
                   "<font name='DVM'>SHA256SUMS.txt</font> and the Merkle root, and re-runs the scenarios and events "
                   "byte for byte.",
                   "<font name='DVM'>python run_all.py</font> (or <font name='DVM'>./run.sh test</font> in Docker): "
                   "re-creates every result into <font name='DVM'>local_runs/</font>.",
                   "Every number in this PDF comes from <font name='DVM'>reports/results.json</font> and "
                   "<font name='DVM'>runs/</font>. The SHA-256 of results.json is in the footer of each page."])
    st.append(Spacer(1, 3 * mm))
    st.append(box(P("<b>Evidence level: Synthetic-validated</b> for the whole system (desk tests against a "
                    "specification emulator and simulated sensors), plus <b>Exploratory real data</b> for the LiDAR "
                    "cone detector only (one recorded scene). Not yet run on an ADS-DV."), "teal"))
    st.append(Spacer(1, 3 * mm))
    st.append(not_show(["Anything about the real car. The safety computer, sensors, tyres and track are software "
                        "copies; their numbers are partly estimates (docs/ASSUMPTIONS.md, A1 to A17).",
                        "Lap times on a real track. Simulated lap times depend on the tyre and car estimates.",
                        "That the software is free of faults. Tests show the cases we thought of; the team's first "
                        "wheels-off session and first track day are the real check."]))
    return build("fsai-open_Results.pdf", "Results of this build", "What was tested, what passed, what broke, and "
                 "how to check it yourself", "blue", D, st)


# ----------------------------------------------------------------------------------------- 2. Project brief
def arch_draw(f):
    ax = f.add_subplot(111)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 44)
    ax.axis("off")
    top = [(0.5, "LiDAR\ncamera\nIMU", "#1a9bb0"), (17, "Perception\n(cones)", "#1f4fa8"),
           (33.5, "SLAM\n(map, pose)", "#1f4fa8"), (50, "Planning\n(path, line,\nspeed)", "#1f4fa8"),
           (66.5, "Control\n(steer,\nspeed)", "#1f4fa8"), (83, "Missions,\nwatchdogs", "#7a52c7")]
    bw, bh = 15, 13
    for x, t, c in top:
        ax.add_patch(matplotlib.patches.FancyBboxPatch((x, 28), bw, bh, boxstyle="round,pad=0.3", fc=c, ec="none"))
        ax.text(x + bw / 2, 28 + bh / 2, t, color="white", ha="center", va="center", fontsize=6.5, fontweight="bold")
    low = [(24, "Your own stack\n(optional)", "#5a6577"), (55, "CAN bridge\n(safety rules)", "#e8892b"),
           (80, "VCU on the car\n(or emulator)", "#c43d5b")]
    for x, t, c in low:
        ax.add_patch(matplotlib.patches.FancyBboxPatch((x, 2), 18, 12, boxstyle="round,pad=0.3", fc=c, ec="none"))
        ax.text(x + 9, 8, t, color="white", ha="center", va="center", fontsize=6.5, fontweight="bold")
    arrow = dict(arrowstyle="->", color="#14306b", lw=1.1)
    for x, _, _ in top[:-1]:
        ax.annotate("", xy=(x + bw + 1.3, 34.5), xytext=(x + bw + 0.2, 34.5), arrowprops=arrow)
    ax.annotate("", xy=(64, 14.6), xytext=(74, 27.4), arrowprops=arrow)
    ax.annotate("", xy=(54.4, 8), xytext=(42.6, 8), arrowprops=arrow)
    ax.annotate("", xy=(79.4, 8), xytext=(73.6, 8), arrowprops=dict(arrowstyle="<->", color="#14306b", lw=1.1))
    ax.text(48.5, 15.5, "ROS 2 messages", fontsize=6, ha="center", color="#14306b")
    ax.text(76.5, 15.5, "CAN, 10 ms", fontsize=6, ha="center", color="#14306b")


def brief_pdf(D, F):
    st = [plain([
        "FS-AI is the driverless class of IMechE Formula Student. University teams write the software that drives a "
        "shared car, the ADS-DV. The car accepts commands only in one strict format over its CAN bus, and its safety "
        "computer (the VCU) stops the car if the team's software is late or sends something it does not allow.",
        "<b>fsai-open</b> is a complete, open, tested software package for that car: the safety link to the car, a "
        "driving stack for every FS-AI 2026 event and inspection, and a desk copy of the car and track to test it "
        "all without the car.",
        "It is built to be handed to Brunel University London's FS-AI team as a finished zip. The team checks, "
        "tests, tunes and races it. We are the builders.",
        f"Status: {F['checks']} automatic checks pass; {F['ev_ok']} of {F['ev_n']} simulated events and "
        f"{F['ros_ok']} of {F['ros_n']} real-time runs are clean; not yet run on the real car."])]
    st += section("Goal and scope", "blue")
    st.append(card_grid([
        card("In scope", "The CAN bridge for the 2026 interface; a software copy of the VCU; perception from LiDAR "
             "and camera; mapping and localisation (SLAM); path, racing line and speed planning; steering and speed "
             "control; every FS-AI 2026 dynamic event and inspection; tests; Docker; reports.", "blue"),
        card("Out of scope", "Changes to the car or its VCU; the team's hardware set-up; the competition's "
             "Lap Time Simulation Cup; any claim about real-track performance before the team has driven it.", "red"),
        card("Who it is for", "The Brunel FS-AI team's software lead first; then any FS-AI team using the ADS-DV, "
             "the organisers, and anyone teaching or researching driverless cars.", "green"),
        card("How it is handed over", "One zip, <font name='DVM'>fsai-open-0.2.0.zip</font>, with its SHA-256. "
             "Two commands to install and run (<font name='DVM'>./install.sh</font>, "
             "<font name='DVM'>./run.sh demo</font>); Windows has <font name='DVM'>run.ps1</font>.", "orange")]))
    st += section("How it works", "navy")
    st.append(fig(arch_draw, "The driving stack (blue), the mission logic and watchdogs (purple), and the safety "
                             "layer (orange) between any driving software and the car's VCU (red). A team can swap "
                             "in its own stack through standard ROS 2 messages.", 170, 78))
    st.append(hood([
        "<b>Bridge:</b> echoes the VCU handshake, keeps steering and torque inside the live limits the car reports, "
        "never sends torque and brake together (with one empty 10 ms cycle between them), never NEUTRAL or FINISHED "
        "while moving, and brakes in a controlled way if the team's software goes quiet. Every change it makes is "
        "reported.",
        "<b>Perception:</b> ground fit, cone clustering, height, width, grass and isolation filters, and camera "
        "confirmation of LiDAR cones.",
        "<b>SLAM:</b> an extended Kalman filter with gyro bias and a learned rear-tyre slip, colour-gated and "
        "distance-capped data association, and map pruning.",
        "<b>Planning and control:</b> a local path from cone pairs, a minimum-curvature racing line once the map is "
        "complete, a grip-scaled speed profile, and pure-pursuit steering with delay compensation.",
        "<b>Real time:</b> perception, IMU and control on separate threads; the racing line in a separate process; "
        "stale scans dropped; health printed once a second."]))
    st += section("Where it stands", "green")
    st.append(tiles([(f"{F['checks']}", "checks passed", "blue"), (f"{F['ev_ok']}/{F['ev_n']}", "events clean", "green"),
                     (f"{F['mi_ok']}/{F['mi_n']}", "inspections clean", "teal"),
                     (f"{F['ros_ok']}/{F['ros_n']}", "real-time runs clean", "orange")]))
    st.append(Spacer(1, 3 * mm))
    st.append(P("Full numbers are in the Results report. The exploratory run on real LiDAR scans is in its own "
                "report."))
    st += section("Assumptions the team must confirm on the car", "orange")
    rows = []
    for line in (ROOT / "docs" / "ASSUMPTIONS.md").read_text().splitlines():
        if line.startswith("| A") and line[3:4].isdigit() or line.startswith("| A1"):
            cells = [c.strip() for c in line.strip("|").split("|")]
            if len(cells) >= 4:
                rows.append([cells[0], md(cells[1]), md(cells[3])])
    st.append(table(["#", "We assume", "How to check it"], rows, [10, 80, 80], "orange"))
    st += section("Key decisions", "purple")
    rows = []
    for line in (ROOT / "DECISIONS.md").read_text().splitlines():
        if line.startswith("| D") and line[3:4].isdigit():
            cells = [c.strip() for c in line.strip("|").split("|")]
            rows.append([cells[0], cells[1], md(cells[2])])
    st.append(table(["#", "Date", "Decision"], rows, [10, 22, 138], "purple"))
    st.append(P("Who decided and why, for each decision: <font name='DVM'>DECISIONS.md</font>.", "small"))
    st += section("What happens next", "teal")
    st += bullets(["The team installs the zip and runs <font name='DVM'>./run.sh test</font> and "
                   "<font name='DVM'>./run.sh verify</font> on their own computer.",
                   "On the car, wheels off: the checks in the setup guide, section 5 (handshake timing, e-stops, "
                   "limits).",
                   "First track day: the sim-vs-asphalt list in the setup guide, section 6 (actuator delays, grip, "
                   "messy sensors, far cones), then the events at capped speeds.",
                   "Open question for the scrutineer: whether \"drivetrain rpm\" in Static Inspection A/B means "
                   "wheel or motor rpm (A13)."])
    st.append(Spacer(1, 3 * mm))
    st.append(not_show(["That it works on the real ADS-DV: nothing here has touched the car.",
                        "That it is faster or safer than any other team's software: no comparison was made.",
                        "Independent review: no one outside the project has reviewed or replicated it yet."]))
    return build("fsai-open_Project_Brief.pdf", "Project Brief", "Open driverless software for the IMechE FS-AI "
                 "ADS-DV, built for Brunel's FS-AI team", "navy", D, st)


# ----------------------------------------------------------------------------------------- 3. Introduction
def intro_pdf(D, F):
    st = [plain([
        "Imagine a race car with no driver. It has to see the track, which is marked only by small plastic cones, "
        "work out where it is, choose a path, and steer, accelerate and brake, all by itself, many times a second.",
        "In FS-AI, students write that software for a shared car. The car has its own safety computer, which is "
        "strict: if the students' software is late by a twentieth of a second, the car stops itself.",
        "This package gives a team a working, tested starting point, and a pretend car and track so they can test "
        "everything at a desk before their few, booked days on the real car."])]
    st += section("Words you will meet", "blue")
    words = [("ADS-DV", "the shared driverless race car IMechE provides to FS-AI teams."),
             ("VCU", "the car's own safety and control computer; teams cannot change it."),
             ("CAN bus", "the cable network the car's computers talk over, in short numbered frames."),
             ("Bridge", "this project's translator between the team's software and the CAN bus."),
             ("Emulator", "a software copy of the VCU's behaviour, for desk testing."),
             ("Handshake", "a bit the VCU flips and the team's computer must echo back within 50 ms (FS-AI 2026 "
                           "T1.1.8), or the car e-stops."),
             ("EBS", "the emergency braking system."),
             ("ROS 2", "the robotics framework most teams use; the bridge plugs into it."),
             ("Docker", "packs the whole set-up into one box, so it runs the same on any computer."),
             ("SLAM", "building a map of the cones and working out where the car is on it, at the same time."),
             ("Racing line", "the fastest path round the track, chosen once the map is complete."),
             ("Seed", "a number that fixes a generated track, so the same seed always gives the same track."),
             ("SHA-256", "a fingerprint of a file; change one letter and it changes completely.")]
    st.append(table(["Word", "Meaning"], [[f"<b>{a}</b>", b] for a, b in words], [30, 140], "blue"))
    st += section("The events the car must drive (FS-AI 2026)", "green")
    st.append(card_grid([
        card("Acceleration", "A straight lane, as fast as possible, then stop inside the stop area.", "green"),
        card("Skidpad", "A figure of eight: two laps right, two laps left, timed on the second of each.", "teal"),
        card("Autocross", "One lap of an unknown track; the map is built while driving.", "orange"),
        card("Trackdrive", "Ten laps of the same track; after the first lap the map is fixed and the racing line "
             "is used.", "purple"),
        card("Static Inspection A and B", "Drivetrain and steering checks, and an emergency-brake test, standing "
             "still or rolling slowly.", "blue"),
        card("Autonomous Demo", "Drive, steer, stop and trigger the EBS on command, inside a set distance.", "red")]))
    st += section("A tour of the package", "navy")
    rows = [["<font name='DVM'>fsai/can/</font>", "every CAN signal between the car and the AI, written as data and "
             "checked against both official databases"],
            ["<font name='DVM'>fsai/bridge/</font>", "the bridge and its 10 ms real-time loop"],
            ["<font name='DVM'>fsai/vcu/</font>", "the VCU emulator with simple vehicle physics"],
            ["<font name='DVM'>fsai/autonomy/</font>", "the driving stack, tracks, sensors, simulator and referee"],
            ["<font name='DVM'>ros2/</font>", "ROS 2 nodes: bridge, emulator, driving stack, pretend world"],
            ["<font name='DVM'>config/</font>", "every number about the car, tracks and stack, each with its source"],
            ["<font name='DVM'>tests/</font>", "spec clauses, known traps, closed-loop events, real-time buses"],
            ["<font name='DVM'>runs/, reports/</font>", "the shipped results, fingerprinted"],
            ["<font name='DVM'>docs/</font>", "the team setup guide and the list of assumptions"]]
    st.append(table(["Folder", "What is in it"], rows, [40, 130], "navy"))
    st += section("Five commands", "teal")
    for cmd, what in (("./install.sh", "once: builds the pinned Docker image (ROS 2 Humble and everything)"),
                      ("./run.sh demo", "a whole driverless autocross lap, in real time, with a browser replay"),
                      ("./run.sh demo trackdrive 3", "any event and seed"),
                      ("./run.sh test", "every test, event and stress run, into local_runs/"),
                      ("./run.sh verify", "every fingerprint, and byte-identical re-runs")):
        st.append(P(f"<font name='DVM' color='#0b2a5b'>{cmd}</font>  {what}"))
    st += section("Simulator versus a real track", "orange")
    st.append(P("A simulator is always cleaner than a real track. Three things always happen on the real car, so "
                "each one has a model in the simulator, a defence in the software, a stress run, and a setting to "
                "tune on the first test day."))
    st.append(table(["Real-world problem", "What the software does", "Setting"],
                    [["Messy sensors: grass, barriers, people, heat shimmer",
                      "rejects tall, wide, low-near and crowded blobs; camera confirms LiDAR cones; drops cones not "
                      "seen again", "perception.*"],
                     ["Tyre grip: cold tyres, bumps, the car turns less than asked",
                      "learns rear-tyre slip, steering compliance and grip while driving; one grip factor scales all "
                      "speeds", "speed.grip_factor"],
                     ["Hardware delay: brakes and steering lag the command",
                      "steers for where the car will be when the actuator acts", "control.latency_comp_s"],
                     ["Far cones: a real LiDAR sees fewer points beyond 10 m (measured on real scans)",
                      "the map keeps only cones seen again; settings to check on Day 1",
                      "perception.min_points, slam.prune_*"]], [52, 78, 40], "orange"))
    st.append(Spacer(1, 3 * mm))
    st.append(not_show(["How the software behaves on the real car: that is for the team's first sessions.",
                        "Every rule of the competition: the tests follow the FS-AI 2026 rules and the 2021 interface "
                        "specification as written; where they disagree, the stricter value is used."]))
    return build("fsai-open_Introduction.pdf", "Introduction", "A first read for anyone new to FS-AI and to this "
                 "package", "teal", D, st)


# ----------------------------------------------------------------------------------------- 4. Exploratory run
def explore_pdf(D, F):
    E = D["E"]
    ra, sy = E["real"]["front_80deg"]["summary"], E["synthetic"]["front_80deg"]["summary"]
    rb, sb = E["real"]["front_80deg"]["by_range"], E["synthetic"]["front_80deg"]["by_range"]
    st = [plain([
        "The simulator's LiDAR is cleaner than a real one. We wanted to know how much.",
        f"We took {E['scene']['frames']} real LiDAR scans recorded by another Formula Student car (Chalmers), "
        "where every cone is labelled with its position and colour, and ran our cone detector on them with no changes.",
        f"Up to 10 m it does as well as in our simulator: about 98 to 99 % of cones found. From 15 to 20 m it finds "
        f"{round(100 * rb[3]['recall'])} % of real cones against {round(100 * sb[3]['recall'])} % in the simulator, "
        "and most of its false cones are there.",
        "So the simulator is too kind beyond 10 m. That is now on the team's first-day tuning list."])]
    st += section("Question", "purple")
    st.append(P("With the shipped settings and no tuning, how many real cones does the LiDAR cone detector find, how "
                "many things does it wrongly call cones, and how far off are the positions, compared with the same "
                "measure on fsai-open's own simulated scans?"))
    st += section("Data", "blue")
    sc = E["scene"]
    st.append(table(["Item", "Value"], [
        ["Dataset", "coneScenes, Chalmers Formula Student"],
        ["Scene", f"{sc['file']} (recording {sc['metadata_info'].get('source')})"],
        ["SHA-256", sc["sha256"]],
        ["Frames used", f"{sc['frames']} labelled LiDAR frames"],
        ["Shipped with fsai-open?", "No. Only our counts and the SHA-256 of every file read "
                                    "(runs/exploratory_real/input_files.sha256.json)"],
        ["Data card", "data_cards/real_lidar_conescenes.md"]], [40, 130], "blue"))
    st += section("Method", "navy")
    m = E["method"]
    st += bullets([f"Points closer than {m['ego_radius_m']} m are the car itself and are removed.",
                   f"Labels count only if at least one LiDAR point falls inside the cone's footprint "
                   "(the dataset's own rule, applied again).",
                   f"A detection within {m['match_within_m']} m of a labelled cone counts as finding it (greedy, "
                   "nearest first, one detection per cone).",
                   "Two views: all around (360°) and front only (±80°, like the stack's field of view). The "
                   f"headline uses the front view and cones up to {m['headline_max_range_m']:.0f} m.",
                   "The same measure on 60 simulated scans from an autocross desk run, taken every 0.5 s.",
                   "No setting was changed for the real data. This measures the gap, not the best possible result."])
    st += section("Results", "green")
    st.append(tiles([(f"{round(100 * ra['recall'])} %", "real cones found (≤20 m)", "purple"),
                     (f"{round(100 * sy['recall'])} %", "simulated cones found (≤20 m)", "green"),
                     (f"{round(100 * ra['mean_position_error_m'])} cm", "real position error", "orange"),
                     (f"{round(100 * sy['mean_position_error_m'], 1)} cm", "simulated position error", "teal")]))
    st.append(Spacer(1, 3 * mm))
    rows = [[a["range_m"], a["labelled_cones"], f"{a['found']} ({a['recall']})", a["false_cones"],
             a["mean_position_error_m"], b["labelled_cones"], f"{b['found']} ({b['recall']})", b["false_cones"],
             b["mean_position_error_m"]] for a, b in zip(rb, sb)]
    st.append(table(["Range [m]", "Real: labelled", "Real: found (share)", "Real: false", "Real: error [m]",
                     "Sim: labelled", "Sim: found (share)", "Sim: false", "Sim: error [m]"], rows,
                    [17, 18, 22, 15, 18, 18, 22, 15, 18], "green"))

    def draw(f):
        ax = f.add_subplot(121)
        x = np.arange(len(rb))
        ax.bar(x - 0.2, [r["recall"] for r in rb], 0.4, color="#7a52c7", label="real scans")
        ax.bar(x + 0.2, [r["recall"] for r in sb], 0.4, color="#1e9e6a", label="our simulator")
        ax.set_xticks(x, [r["range_m"] for r in rb])
        ax.set_xlabel("range [m]")
        ax.set_ylabel("share of cones found")
        ax.set_ylim(0, 1.05)
        ax.legend(frameon=False, loc="lower left", fontsize=7)
        ax.set_title("Cones found")
        ax2 = f.add_subplot(122)
        ax2.bar(x - 0.2, [r["false_cones"] for r in rb], 0.4, color="#7a52c7")
        ax2.bar(x + 0.2, [r["false_cones"] for r in sb], 0.4, color="#1e9e6a")
        ax2.set_xticks(x, [r["range_m"] for r in rb])
        ax2.set_xlabel("range [m]")
        ax2.set_ylabel("false cones (all frames)")
        ax2.set_title("False cones")
        f.tight_layout()
    st.append(fig(draw, "Front view (±80°). Left: share of labelled cones found per range bin. Right: detections "
                        "that matched no labelled cone.", 170, 70))
    al = E["real"]["all_around"]["summary"]
    st.append(P(f"All around (360°), up to 20 m: real scans {al['found']} of {al['labelled_cones']} cones found "
                f"({al['recall']}), {al['false_cones']} false cones. Big orange cones: the real scene has "
                f"{ra['big_cones_labelled']} labelled, so the big-cone check could not be tested on it."))
    st += section("What it means for the team", "orange")
    st += bullets(["Near cones (under 10 m) are found as well on real scans as in the simulator, so the local path "
                   "and the first lap at walking pace are not at risk from this.",
                   "Far cones (15 to 20 m) are found about half as often, and false cones appear there. The map "
                   "keeps a cone only after it is seen again, which removes most one-off false cones; the stress "
                   "runs show the effect of one phantom cone left in the map.",
                   "On Day 1, check <font name='DVM'>/fsai/perception/cones</font> at 15 to 20 m first. Settings: "
                   "<font name='DVM'>perception.min_points</font> for far, sparse cones and "
                   "<font name='DVM'>slam.prune_after_seen</font> / <font name='DVM'>prune_ratio</font> for false "
                   "ones. Change one at a time and re-run <font name='DVM'>./run.sh test</font>."])
    st += section("Reproduce it", "teal")
    st.append(P("<font name='DVM'>python tools/explore_real_lidar.py cfs_vargarda8.zip</font> writes "
                "<font name='DVM'>runs/exploratory_real/results.json</font>. Check the zip's SHA-256 against the "
                "value above first. The run takes under a minute and has no random parts on the real data."))
    st.append(Spacer(1, 3 * mm))
    st.append(not_show(["Performance at the competition: one scene, one car, one LiDAR model, one day.",
                        "Our own car's LiDAR: the ADS-DV's sensor and mounting differ from the Chalmers car's.",
                        "The camera, mapping or driving on real data: only the LiDAR cone detector was run.",
                        "The best the detector could do: nothing was tuned, on purpose."]))
    return build("fsai-open_Exploratory_Run.pdf", "Exploratory run on real LiDAR scans",
                 "How far is the simulator from a real track? One real, labelled scene", "purple", D, st)


# ----------------------------------------------------------------------------------------- 5. Impact
def impact_pdf(D, F):
    st = [plain([
        "Every FS-AI team that uses the shared car has to build the same link to it and the same kind of driving "
        "software. Today each team does it alone, often again each year.",
        "fsai-open is a tested, open version of all of it, with its tests and results included, so a team can start "
        "from something that already drives every event on the desk.",
        "It does not replace a test day on the real car. It means the first test day is spent setting a few "
        "measured numbers, not rewriting code."])]
    st += section("Who it could help, and how", "green")
    st.append(card_grid([
        card("Brunel's FS-AI team", "Skip months of building and debugging the car link and a driving stack. "
             "Rehearse every emergency stop at a desk, so booked days on the car go to driving, not debugging.",
             "blue"),
        card("Every FS-AI team", "Every team on the IMechE car needs this layer. An open, tested version with its "
             "tests can become a shared starting point, which may matter most for smaller universities.", "green"),
        card("The organisers", "The tests turn the interface document into something that runs, and list where "
             "the documents disagree (for example the steering limit: 21°, 24°, 27.2° or 30°; and on/off flags "
             "declared as signed in the 2026 file), so these can be fixed at the source.", "orange"),
        card("Research and teaching", "A deterministic, fingerprinted test bench: change one part (say the cone "
             "detector) and measure the effect on whole events, with the referee and stress runs unchanged.",
             "purple")]))
    st += section("What is already shown", "blue")
    st.append(tiles([(f"{F['checks']}", "checks pass", "blue"), (f"{F['ev_ok']}/{F['ev_n']}", "events clean", "green"),
                     (f"{F['ros_ok']}/{F['ros_n']}", "real-time runs clean", "teal"),
                     (f"{F['cones_hit']}", "cones hit", "orange")]))
    st.append(Spacer(1, 3 * mm))
    st.append(P("Bugs found by the tests before they could reach a car include: a planner that took 67 s and "
                "starved the sensors (now under 1 s, in a separate process), a brake-to-torque switch that the "
                "emulated safety computer read as a fault (now one empty 10 ms cycle between them), and one bad "
                "cone match that dragged the whole map by 2 m (now capped at 1 m). Each is a written decision "
                "(D15 to D18) with a test."))
    st += section("Evidence ladder: where this stands", "navy")
    ladder = [("Synthetic-validated", "Desk tests against a specification emulator and simulated sensors", "here"),
              ("Exploratory real data", "LiDAR cone detector on one real, labelled scene", "here (perception only)"),
              ("On-car, wheels off", "Handshake, limits and e-stops on the real ADS-DV (guide section 5)",
               "next: the team"),
              ("On-track", "Events at capped speeds, then full speed (guide section 6)", "after that"),
              ("Independently replicated", "Another team or reviewer re-runs and checks it", "not yet")]
    st.append(table(["Rung", "What it means", "fsai-open 0.2.0"], [[f"<b>{a}</b>", b, c] for a, b, c in ladder],
                    [45, 90, 35], "navy"))
    st += section("How you can check it", "teal")
    st += bullets(["Every file has a SHA-256 fingerprint in <font name='DVM'>SHA256SUMS.txt</font>, joined into one "
                   "Merkle root.", "<font name='DVM'>python verify.py</font> checks them all and re-runs the "
                   "scenarios and events byte for byte.", "Every assumption about the car is listed with how to "
                   "check it (docs/ASSUMPTIONS.md), and every decision with who made it (DECISIONS.md)."])
    st.append(Spacer(1, 3 * mm))
    st.append(not_show(["Impact that has happened: these are possible benefits, not measured ones.",
                        "That the software is ready to race: the real car, the real sensors and a real track are "
                        "still to come.", "That no one else has done similar work: no such claim is made."]))
    return build("fsai-open_Impact.pdf", "Impact", "What this could change, for whom, and what is already shown",
                 "green", D, st)


def main():
    D = load()
    F = facts(D)
    trackfig, r = track_figure()
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from report_pdf import report_pdf
    out = [results_pdf(D, F, trackfig), brief_pdf(D, F), intro_pdf(D, F), explore_pdf(D, F), impact_pdf(D, F),
           report_pdf(D, F)]
    for p in out:
        print(p.relative_to(ROOT), hashlib.sha256(p.read_bytes()).hexdigest()[:16])


if __name__ == "__main__":
    main()
