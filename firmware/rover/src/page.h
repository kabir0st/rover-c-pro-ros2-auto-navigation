// Control page served at "/". Plain HTML + JS, no external files, works offline.
//
// Hold a button to drive; release to stop. While held, the page repeats the
// command every 150 ms, and the rover stops by itself if commands stop arriving
// (tab closed, WiFi drop), so nothing keeps driving unattended.
//
// Directions: each button's data-x / data-y / data-z go straight into
// M5_RoverC::setSpeed(x, y, z): y+ = forward, x+ = strafe right,
// z+ = turn left (counter-clockwise). If a direction is mirrored on your rover,
// flip the sign on that button.
#pragma once

static const char PAGE[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>RoverC</title>
<style>
:root{color-scheme:dark}
body{margin:0;padding:16px;background:#111;color:#eee;font-family:system-ui,sans-serif;
  display:flex;flex-direction:column;align-items:center;gap:16px;
  user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}
h1{margin:0;font-size:20px}
.pad{display:grid;grid-template-columns:repeat(3,80px);gap:8px}
.rot{display:flex;gap:8px}
.rot button{width:124px}
button{height:80px;border:0;border-radius:12px;background:#333;color:#eee;font-size:30px;touch-action:none}
button.on{background:#2a7}
#stop{background:#a33}
label{display:flex;flex-direction:column;gap:6px;width:256px;font-size:14px}
#st{font-size:14px;color:#aaa;min-height:1.2em}
</style></head><body>
<h1>RoverC</h1>
<div class="pad">
  <button data-x="-1" data-y="1">&#8598;</button>
  <button data-y="1">&#8593;</button>
  <button data-x="1" data-y="1">&#8599;</button>
  <button data-x="-1">&#8592;</button>
  <button id="stop">&#9632;</button>
  <button data-x="1">&#8594;</button>
  <button data-x="-1" data-y="-1">&#8601;</button>
  <button data-y="-1">&#8595;</button>
  <button data-x="1" data-y="-1">&#8600;</button>
</div>
<div class="rot">
  <button data-z="1">&#10226;</button>
  <button data-z="-1">&#10227;</button>
</div>
<label>Speed: <span id="sv">60</span>%<input id="speed" type="range" min="20" max="100" value="60"></label>
<label>Gripper<input id="grip" type="range" min="10" max="90" value="50"></label>
<div id="st">ready</div>
<script>
const st = document.getElementById('st');
const speed = document.getElementById('speed');
const sv = document.getElementById('sv');
const grip = document.getElementById('grip');
const STOP = 'x=0&y=0&z=0';
let cmd = STOP, timer = null, busy = false;

function send(force) {
  if (busy && !force) return;   // don't pile up requests on a slow link
  busy = true;
  fetch('/drive?' + cmd)
    .then(r => r.text()).then(t => st.textContent = t)
    .catch(() => st.textContent = 'no connection to rover')
    .finally(() => busy = false);
}
function go(b) {
  const s = +speed.value, d = b.dataset;
  cmd = `x=${(d.x || 0) * s}&y=${(d.y || 0) * s}&z=${(d.z || 0) * s}`;
  b.classList.add('on');
  send(true);
  clearInterval(timer);
  timer = setInterval(send, 150);
}
function halt(b) {
  if (b) b.classList.remove('on');
  clearInterval(timer);
  timer = null;
  cmd = STOP;
  send(true);
}
document.querySelectorAll('button').forEach(b => {
  b.addEventListener('pointerdown', e => { b.setPointerCapture(e.pointerId); go(b); });
  ['pointerup', 'pointercancel', 'lostpointercapture']
    .forEach(ev => b.addEventListener(ev, () => halt(b)));
  b.addEventListener('contextmenu', e => e.preventDefault());
});
window.addEventListener('blur', () => halt());
document.addEventListener('visibilitychange', () => { if (document.hidden) halt(); });
// Check in once a second so the rover's screen knows this page is open.
// The reply also keeps the lock status here up to date while not driving.
function ping() {
  fetch('/ping')
    .then(r => r.text()).then(t => { if (!timer) st.textContent = t; })
    .catch(() => st.textContent = 'no connection to rover');
}
ping();
setInterval(ping, 1000);
speed.oninput = () => sv.textContent = speed.value;
grip.onchange = () => fetch('/grip?angle=' + grip.value);
</script>
</body></html>
)HTML";
