"""Uji halaman web firmware di browser SUNGGUHAN (Chrome/Edge headless + CDP) dgn ESP32 tiruan.
    python uji_browser.py mouse   |   python uji_browser.py touch   (tampilan & sentuhan HP)

- HTML diambil langsung dari INDEX_HTML di dep_frayer.ino, disajikan di http://127.0.0.1:8099/
- Server WebSocket tiruan di 127.0.0.1:81 (port sama dgn ESP32): kirim status 'st' tiap 250 ms
  dan 'temp' tiap 1 s dgn format firmware rev 24, catat semua perintah yang diterima.
- Chrome headless: buka halaman, kumpulkan error JS/console, lalu KLIK MOUSE SUNGGUHAN
  (Input.dispatchMouseEvent) di tengah setiap tombol dan cek perintah apa yang sampai ke ESP.
"""
import base64, hashlib, http.server, json, os, re, socket, struct, subprocess, sys, tempfile, threading, time
import urllib.request
import websocket  # websocket-client (CDP)
sys.stdout.reconfigure(encoding="utf-8", errors="replace")

INO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dep_frayer", "dep_frayer.ino")
CHROME = next((c for c in (r"C:\Program Files\Google\Chrome\Application\chrome.exe",
                           r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe") if os.path.exists(c)), None)
src = open(INO, encoding="utf-8").read()
HTML = src[src.index('R"rawliteral(') + len('R"rawliteral('):src.index(')rawliteral"')]

# ---------------- HTTP ----------------
class H(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        b = HTML.encode()
        self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
    def log_message(self, *a): pass
httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 8099), H)
threading.Thread(target=httpd.serve_forever, daemon=True).start()

# ---------------- WebSocket ESP tiruan ----------------
DITERIMA = []
ST = {"type":"st","ls":[0,0,0,0],"raw":[1,1,1,1],"vr":[1,1,1,1],"il":[1,1,1,1],"pn":[17,35,39,34],
      "bn":0,"dx":0,"xp":0,"xm":0,"en":0,"sx":3000,"ax":3000,"sa":-3,"sg":-3,"sr":0,"sl":0,"um":500,"ux":2500,
      "cd":-3,"od":55,"tc":180,"dcc":0,"dct":0,"dcm":50,"dcv":24,"sp":160.0,"kp":1.46,"ki":0.0054,"kd":0.0,
      "pm":100.0,"ad":1,"lp":0.06,"li":0.0,"ld":5.0,"fw":"rev 27","rr":"POWER_ON","hp":180000,"sk":3000,"lr":0,"wc":1,"wm":5}
def temp(seq):
    return {"type":"temp","t":150.25,"ok":True,"raw":"09640000","d":"frame valid","up":seq,"sp":160.0,
            "seq":seq,"ms":seq*1000,"st":0,"te":0,"tr":150.3,"held":0,"ec":0,"pe":0,"pu":0.0,"se":None,
            "kpe":1.46,"kie":0.0054,"kde":0.0,"ph":0}

def frame(t):
    d = t.encode(); n = len(d)
    hdr = bytes([0x81, n]) if n < 126 else bytes([0x81, 126]) + struct.pack(">H", n)
    return hdr + d

def baca_frame(c):
    h = c.recv(2)
    if len(h) < 2: return None
    n = h[1] & 0x7F
    if n == 126: n = struct.unpack(">H", c.recv(2))[0]
    elif n == 127: n = struct.unpack(">Q", c.recv(8))[0]
    mask = c.recv(4); d = b""
    while len(d) < n: d += c.recv(n - len(d))
    d = bytes(b ^ mask[i % 4] for i, b in enumerate(d))
    return (h[0] & 0x0F, d)

def layani(c):
    req = c.recv(4096).decode()
    key = re.search(r"Sec-WebSocket-Key: (.+)\r\n", req).group(1).strip()
    acc = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
    c.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               "Sec-WebSocket-Accept: %s\r\n\r\n" % acc).encode())
    hidup = [True]
    def pembaca():
        while hidup[0]:
            try:
                f = baca_frame(c)
            except OSError:
                break
            if f is None or f[0] == 8: break
            if f[0] == 1: DITERIMA.append(f[1].decode())
        hidup[0] = False
    threading.Thread(target=pembaca, daemon=True).start()
    seq = 0; k = 0
    while hidup[0]:
        try:
            c.sendall(frame(json.dumps(ST)))
            if k % 4 == 0:
                seq += 1; c.sendall(frame(json.dumps(temp(seq))))
        except OSError:
            break
        k += 1; time.sleep(0.25)

def ws_server():
    s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", 81)); s.listen(4)
    while True:
        c, _ = s.accept(); threading.Thread(target=layani, args=(c,), daemon=True).start()
threading.Thread(target=ws_server, daemon=True).start()

# ---------------- Chrome headless + CDP ----------------
prof = tempfile.mkdtemp()
chrome = subprocess.Popen([CHROME, "--headless=new", "--remote-debugging-port=9223", "--user-data-dir=" + prof,
                           "--no-first-run", "--disable-gpu", "--window-size=1000,4000", "about:blank"],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
for _ in range(50):
    try:
        tabs = json.load(urllib.request.urlopen("http://127.0.0.1:9223/json")); break
    except Exception:
        time.sleep(0.2)
page = next(t for t in tabs if t["type"] == "page")
cdp = websocket.create_connection(page["webSocketDebuggerUrl"], timeout=10)
_id = [0]; ERR = []; DIALOG = []
def kirim(method, **params):
    _id[0] += 1; cdp.send(json.dumps({"id": _id[0], "method": method, "params": params}))
    while True:
        m = json.loads(cdp.recv())
        if m.get("id") == _id[0]: return m.get("result", m.get("error"))
        olah_event(m)
def olah_event(m):
    me = m.get("method")
    if me == "Runtime.exceptionThrown":
        d = m["params"]["exceptionDetails"]
        ERR.append("EXCEPTION: %s (baris %s)" % (d.get("exception", {}).get("description", d.get("text")), d.get("lineNumber")))
    elif me == "Runtime.consoleAPICalled" and m["params"]["type"] in ("error", "warning"):
        ERR.append("CONSOLE %s: %s" % (m["params"]["type"], " ".join(str(a.get("value", a.get("description"))) for a in m["params"]["args"])))
    elif me == "Log.entryAdded" and m["params"]["entry"]["level"] in ("error", "warning"):
        ERR.append("LOG: " + m["params"]["entry"]["text"])
    elif me == "Page.javascriptDialogOpening":
        DIALOG.append(m["params"]["message"])
def pompa(detik):
    cdp.settimeout(0.1); t_end = time.time() + detik
    while time.time() < t_end:
        try: olah_event(json.loads(cdp.recv()))
        except Exception: pass
    cdp.settimeout(10)
def js(expr):
    r = kirim("Runtime.evaluate", expression=expr, returnByValue=True)
    return r.get("result", {}).get("value")

kirim("Runtime.enable"); kirim("Log.enable"); kirim("Page.enable")
MODE = sys.argv[1] if len(sys.argv) > 1 else "mouse"
if MODE == "touch":
    kirim("Emulation.setDeviceMetricsOverride", width=400, height=860, deviceScaleFactor=2, mobile=True)
    kirim("Emulation.setTouchEmulationEnabled", enabled=True, maxTouchPoints=5)
print("MODE:", MODE)
kirim("Page.navigate", url="http://127.0.0.1:8099/")
pompa(4)
print("== Setelah halaman dimuat 4 dtk")
print("status:", js("document.getElementById('status').textContent"), "| judul fw:", js("(document.getElementById('fw')||{}).textContent"), "| diag:", js("document.getElementById('diag').textContent"))
print("error JS/console:", ERR if ERR else "TIDAK ADA")

# daftar tombol + cek elemen paling atas di titik tengahnya
tombol = js("""(() => { const out=[]; document.querySelectorAll('button').forEach((b,i)=>{
  b.scrollIntoView({block:'center'}); const r=b.getBoundingClientRect(); const x=r.left+r.width/2, y=r.top+r.height/2;
  const top=document.elementFromPoint(x,y); out.push({i, teks:b.textContent.trim().slice(0,40), oc:b.getAttribute('onclick'),
  tertutup: !(top===b || b.contains(top)), penutup: top? (top.tagName+'#'+top.id+'.'+top.className): null}); }); return out; })()""")
print("jumlah tombol:", len(tombol))
tertutup = [t for t in tombol if t["tertutup"]]
print("tombol tertutup elemen lain:", tertutup if tertutup else "TIDAK ADA")

# klik mouse sungguhan tiap tombol (yang tidak mengubah keadaan berbahaya tetap aman: ESP tiruan)
hasil = []
for t in tombol:
    n0 = len(DITERIMA)
    pos = js("(() => { const b=document.querySelectorAll('button')[%d]; b.scrollIntoView({block:'center'}); const r=b.getBoundingClientRect(); return [r.left+r.width/2, r.top+r.height/2]; })()" % t["i"])
    if MODE == "touch":
        kirim("Input.dispatchTouchEvent", type="touchStart", touchPoints=[{"x": pos[0], "y": pos[1]}])
        kirim("Input.dispatchTouchEvent", type="touchEnd", touchPoints=[])
    else:
        for typ in ("mousePressed", "mouseReleased"):
            kirim("Input.dispatchMouseEvent", type=typ, x=pos[0], y=pos[1], button="left", clickCount=1)
    if DIALOG:
        kirim("Page.handleJavaScriptDialog", accept=True); DIALOG.clear()
    pompa(0.4)
    baru = DITERIMA[n0:]
    hasil.append((t["teks"], t["oc"], baru, js("document.getElementById('log').textContent")))
diam = [h for h in hasil if not h[2]]
print("\n== Hasil klik %d tombol: %d mengirim perintah, %d tidak" % (len(hasil), len(hasil) - len(diam), len(diam)))
for teks, oc, baru, log in hasil:
    print("  %-40s -> %s %s" % (teks, baru if baru else "-", ("| log: " + log) if not baru else ""))
print("\nerror JS/console total:", ERR if ERR else "TIDAK ADA")
subprocess.run(["taskkill", "/PID", str(chrome.pid), "/T", "/F"], capture_output=True); httpd.shutdown()
