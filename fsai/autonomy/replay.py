# fsai-open: one-file browser replay of a simulated run (double-click to open, works offline)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: writes a single web page that replays a run: the cones, the car, where the car
thought it was, the racing line, the speed and the steering. No internet or install needed."""
from __future__ import annotations

import json
from pathlib import Path

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>fsai-open replay</title>
<style>
:root{--bg:#0f1419;--panel:#171d24;--ink:#e6e9ee;--mute:#8a94a3;--line:#2a323d;--accent:#ff8a3d}
@media (prefers-color-scheme: light){:root{--bg:#f6f7f9;--panel:#fff;--ink:#1b2129;--mute:#5d6775;--line:#dde2e8;--accent:#d45e12}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{padding:14px 16px;border-bottom:1px solid var(--line)}h1{font-size:17px;margin:0}header p{margin:4px 0 0;color:var(--mute)}
main{display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:12px;padding:12px 16px}
@media (max-width:820px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px}
canvas{width:100%;display:block;border-radius:8px}#map{aspect-ratio:4/3}#chart{height:150px}
.row{display:flex;gap:8px;align-items:center;margin-top:8px}input[type=range]{flex:1}
button{background:var(--accent);color:#fff;border:0;border-radius:6px;padding:6px 12px;font-weight:600;cursor:pointer}
dl{display:grid;grid-template-columns:auto 1fr;gap:3px 10px;margin:0}dt{color:var(--mute)}dd{margin:0;font-variant-numeric:tabular-nums}
.key span{display:inline-block;width:10px;height:10px;border-radius:50%;margin:0 4px 0 10px;vertical-align:-1px}
footer{color:var(--mute);padding:4px 16px 16px;font-size:12px}
</style></head><body>
<header><h1 id="title"></h1><p id="sub"></p></header>
<main><div><div class="card"><canvas id="map"></canvas>
<div class="row"><button id="play">Play</button><input id="t" type="range" min="0" value="0"><span id="clock"></span></div>
<div class="key"><span style="background:#2f6fde"></span>blue cone<span style="background:#e8c300"></span>yellow cone<span style="background:#ff8a3d"></span>orange<span style="background:#7b5cff"></span>racing line<span style="background:#20b26b"></span>car (true)<span style="background:#e04b6a"></span>car (estimated)</div></div>
<div class="card" style="margin-top:12px"><canvas id="chart"></canvas></div></div>
<div class="card"><dl id="facts"></dl></div></main>
<footer id="foot"></footer>
<script>
const D = __DATA__;
const $ = id => document.getElementById(id);
$('title').textContent = `${D.result.event} - seed ${D.result.seed} - ${D.result.final_state}`;
$('sub').textContent = `Track ${D.result.track_length_m} m. Simulated in the fsai-open desk simulator (synthetic, not a real car).`;
$('foot').textContent = D.footer;
const facts = [['Laps (referee)', (D.result.lap_times_referee_s||[]).length], ['Lap times [s]', (D.result.lap_times_referee_s||[]).join(', ')||'-'],
 ['Cones hit', D.result.cones_hit], ['Off track [s]', D.result.off_track_s], ['Max speed [m/s]', D.result.max_speed_mps],
 ['Map cones (true / false)', `${D.result.map_true_cones} / ${D.result.map_false_cones}`], ['Map colour accuracy', D.result.map_colour_accuracy],
 ['Map RMSE [m]', D.result.map_rmse_m], ['Faults', (D.result.faults||[]).join(', ')||'none'], ['Bridge rule breaks', (D.result.bridge_violations||[]).length],
 ['Trace SHA-256', D.result.trace_sha256.slice(0,16)+'...']];
$('facts').innerHTML = facts.map(([k,v])=>`<dt>${k}</dt><dd>${v}</dd>`).join('');
const C = {0:'#2f6fde',1:'#e8c300',2:'#ff8a3d',3:'#ff5a1f'};
const tr = D.trace; $('t').max = tr.length-1;
let xs=[],ys=[]; D.cones.forEach(c=>{xs.push(c[0]);ys.push(c[1])}); tr.forEach(r=>{xs.push(r[2]);ys.push(r[3])});
const bx=[Math.min(...xs)-5,Math.max(...xs)+5], by=[Math.min(...ys)-5,Math.max(...ys)+5];
function fit(cv){const r=cv.getBoundingClientRect(),d=devicePixelRatio||1;cv.width=r.width*d;cv.height=r.height*d;return [cv.getContext('2d'),cv.width,cv.height,d]}
function draw(i){
 const [g,W,H,d]=fit($('map')); const s=Math.min(W/(bx[1]-bx[0]),H/(by[1]-by[0]));
 const ox=(W-s*(bx[1]-bx[0]))/2, oy=(H-s*(by[1]-by[0]))/2;
 const P=(x,y)=>[ox+(x-bx[0])*s, H-oy-(y-by[0])*s];
 const css=getComputedStyle(document.body); g.fillStyle=css.getPropertyValue('--panel'); g.fillRect(0,0,W,H);
 if(D.line){g.strokeStyle='#7b5cff';g.lineWidth=1.5*d;g.beginPath();D.line.forEach((p,k)=>{const [a,b]=P(p[0],p[1]);k?g.lineTo(a,b):g.moveTo(a,b)});g.closePath();g.stroke()}
 D.cones.forEach(c=>{const [a,b]=P(c[0],c[1]);g.fillStyle=C[c[2]];g.beginPath();g.arc(a,b,(c[2]==3?3.2:2.4)*d,0,7);g.fill()});
 g.strokeStyle='#20b26b';g.lineWidth=1.2*d;g.beginPath();for(let k=0;k<=i;k++){const [a,b]=P(tr[k][2],tr[k][3]);k?g.lineTo(a,b):g.moveTo(a,b)}g.stroke();
 const r=tr[i]; for(const [x,y,h,col] of [[r[2],r[3],r[4],'#20b26b'],[r[7],r[8],r[9],'#e04b6a']]){
  const [a,b]=P(x,y); g.save();g.translate(a,b);g.rotate(-h);g.fillStyle=col;g.fillRect(-0.64*s,-0.715*s,2.815*s,1.43*s);g.restore()}
 $('clock').textContent=`t ${r[0].toFixed(1)} s  v ${r[5].toFixed(1)} m/s  steer ${r[6].toFixed(1)}°  lap ${r[11]}  ${r[10]}`;
 const [c2,W2,H2]=fit($('chart')); c2.fillStyle=css.getPropertyValue('--panel');c2.fillRect(0,0,W2,H2);
 const vmax=Math.max(...tr.map(q=>q[5]),1); c2.strokeStyle='#2f6fde';c2.lineWidth=1.5*d;c2.beginPath();
 tr.forEach((q,k)=>{const X=k/(tr.length-1)*W2, Y=H2-q[5]/vmax*(H2-10*d)-5*d; k?c2.lineTo(X,Y):c2.moveTo(X,Y)});c2.stroke();
 c2.strokeStyle=css.getPropertyValue('--accent');c2.beginPath();const X=i/(tr.length-1)*W2;c2.moveTo(X,0);c2.lineTo(X,H2);c2.stroke();
 c2.fillStyle=css.getPropertyValue('--mute');c2.font=`${11*d}px system-ui`;c2.fillText(`speed, max ${vmax.toFixed(1)} m/s`,6*d,14*d);
}
let i=0,timer=null; $('t').oninput=e=>{i=+e.target.value;draw(i)};
$('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;$('play').textContent='Play';return}
 $('play').textContent='Pause';timer=setInterval(()=>{i=(i+1)%tr.length;$('t').value=i;draw(i)},40)};
addEventListener('resize',()=>draw(i)); draw(0);
</script></body></html>"""


def write_replay(world, result: dict, path: Path, footer: str = "") -> Path:
    tr = world.track
    data = {
        "result": result,
        "cones": [[round(float(x), 3), round(float(y), 3), int(c)] for x, y, c in tr.cones],
        "trace": [list(r) for r in world.trace],
        "line": [[round(float(x), 3), round(float(y), 3)] for x, y in world.stack.global_line["line"]]
        if world.stack.global_line else None,
        "footer": footer,
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    blob = json.dumps(data, separators=(",", ":"), default=float).replace("</", "<\\/")
    path.write_text(PAGE.replace("__DATA__", blob), encoding="utf-8")
    return path
