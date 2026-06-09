"""
Smart Car Competition - Diagram Generator
Pure Python stdlib (xml.etree) SVG generation, zero dependencies.
Output: 7 SVG diagrams + parameters table
"""
import xml.etree.ElementTree as ET
import os

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "diagrams")
os.makedirs(OUT_DIR, exist_ok=True)

def svg(w=800, h=500):
    return ET.Element("svg", xmlns="http://www.w3.org/2000/svg",
                       width=str(w), height=str(h), viewBox=f"0 0 {w} {h}")

def save(root, fname):
    path = os.path.join(OUT_DIR, fname)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    print(f"  OK {fname}")
    return path

def mkrect(p, x, y, w, h, rx=6, fill="#f0f0f0", stroke="#999", sw=1.5):
    return ET.SubElement(p, "rect", {"x": str(x), "y": str(y), "width": str(w),
            "height": str(h), "rx": str(rx), "fill": fill, "stroke": stroke, "stroke-width": str(sw)})

def mktext(p, x, y, txt, size=14, fill="#333", anchor="middle", bold=False):
    a = {"x": str(x), "y": str(y), "font-family": "SimHei, Microsoft YaHei, sans-serif",
         "font-size": str(size), "fill": fill, "text-anchor": anchor}
    if bold: a["font-weight"] = "bold"
    e = ET.SubElement(p, "text", a)
    e.text = txt
    return e

def mklines(p, x, y, lines, size=13, fill="#333", anchor="middle", gap=18):
    for i, line in enumerate(lines):
        mktext(p, x, y + i * gap, line, size=size, fill=fill, anchor=anchor)

def mkarrow(p, x1, y1, x2, y2, color="#666", sw=1.5):
    mid = ET.SubElement(p, "defs")
    kid = f"a{x1}_{y1}_{x2}_{y2}".replace(".", "_")
    m = ET.SubElement(mid, "marker", id=kid, markerWidth="10", markerHeight="7",
                       refX="10", refY="3.5", orient="auto")
    ET.SubElement(m, "polygon", points="0 0, 10 3.5, 0 7", fill=color)
    return ET.SubElement(p, "line", x1=str(x1), y1=str(y1), x2=str(x2), y2=str(y2),
        stroke=color, **{"stroke-width": str(sw), "marker-end": f"url(#{kid})"})

# ====================================================================
# 1: Track Layout (600x550)
# ====================================================================
def draw_track():
    s = svg(600, 550)
    M, W = 80, 120
    pts = [(M,M), (M+W,M), (M+W,M+W), (M,M+W), (M,M)]
    for i in range(4):
        ET.SubElement(s, "line", x1=str(pts[i][0]), y1=str(pts[i][1]),
                       x2=str(pts[i+1][0]), y2=str(pts[i+1][1]),
                       stroke="#333", **{"stroke-width": "3"})
    # Dashed diagonals
    for (a,b) in [((M,M),(M+W,M+W)), ((M+W,M),(M,M+W))]:
        ET.SubElement(s, "line", x1=str(a[0]), y1=str(a[1]), x2=str(b[0]), y2=str(b[1]),
                       stroke="#999", **{"stroke-width": "2", "stroke-dasharray": "8,6"})
    # Labels
    labels = [(M-15, M-15, "A"), (M+W+15, M-5, "B"), (M+W+10, M+W+20, "C"), (M-15, M+W+20, "D")]
    for lx, ly, lb in labels:
        mktext(s, lx, ly, lb, size=16, fill="#2196F3", bold=True)
    mktext(s, M-15, M-15, "A", size=16, fill="#4CAF50", bold=True)  # A is start
    mktext(s, M+W/2, M+W+70, "Solid = black tape edge (Task 1/2 line-following)", size=12, fill="#333")
    mktext(s, M+W/2, M+W+92, "Dashed = no-guide diagonal (Task 3/4 inertial nav)", size=12, fill="#999")
    mktext(s, M+W/2, 30, "Track Layout (120cm square)", size=18, bold=True)
    save(s, "01_track_layout.svg")

# ====================================================================
# 2: System Architecture (750x520)
# ====================================================================
def draw_system():
    s = svg(750, 500)
    mkrect(s, 275, 180, 200, 50, fill="#FFF3E0", stroke="#FF9800")
    mklines(s, 375, 195, ["MSPM0G3507", "Cortex-M0+ 80MHz"], size=11, fill="#E65100")
    # Sensors (left)
    sens = [("IR Sensors x8 S0-S7", 40), ("MPU6050 Gyroscope", 120), ("HC-SR04 Ultrasonic", 200),
            ("Encoders x2 AB-Phase", 280), ("Buttons x4 K1-K4", 360)]
    for title, y in sens:
        mkrect(s, 30, y, 170, 36, fill="#E8F5E9", stroke="#4CAF50")
        mktext(s, 115, y+23, title, size=13)
    # Actuators (right)
    acts = [("TB6612 H-Bridge + Motors", 140), ("SSD1306 OLED 128x64", 220), ("VOFA+ Serial Debug", 300)]
    for title, y in acts:
        mkrect(s, 530, y, 200, 36, fill="#E3F2FD", stroke="#2196F3")
        mktext(s, 630, y+23, title, size=13)
    # Arrows
    for y in [58, 138, 218, 298, 378]:
        mkarrow(s, 200, y+18, 275, 205, "#4CAF50")
    for y in [158, 238, 318]:
        mkarrow(s, 475, 205, 530, y+18, "#2196F3")
    mktext(s, 375, 30, "System Hardware Architecture", size=20, bold=True)
    mktext(s, 375, 460, "ISR Priority: GPIOA(0) > TIMA0(1) > TIMG0(2) > SysTick 1ms", size=12, fill="#888")
    save(s, "02_system_arch.svg")

# ====================================================================
# 3: PID Cascade Control (780x480)
# ====================================================================
def draw_pid():
    s = svg(780, 480)
    # Sensor
    mkrect(s, 30, 180, 120, 40, fill="#F3E5F5", stroke="#9C27B0"); mktext(s, 90, 206, "8x IR Sensors", size=12, bold=True)
    # steerPID
    mkrect(s, 200, 130, 130, 85, fill="#FCE4EC", stroke="#E91E63")
    mklines(s, 265, 148, ["steerPID", "Kp=0.9 Ki=0.03 Kd=0.20", "Limit +-80"], size=11, anchor="middle")
    mkarrow(s, 150, 200, 200, 172, "#E91E63")
    # Diff alloc
    mkrect(s, 380, 170, 100, 36, rx=4, fill="#FFF9C4", stroke="#FBC02D"); mktext(s, 430, 193, "Diff Alloc", size=13)
    mkarrow(s, 330, 172, 380, 188, "#666")
    # BASE_SPEED
    mktext(s, 430, 152, "BASE_SPEED=80", size=11, fill="#888")
    mkarrow(s, 430, 152, 430, 170, "#888", sw=1)
    # leftPID / rightPID
    for x, name, params, col in [
        (150, "leftPID", ["Kp=3.7 Ki=0.30 Kd=0.05", "Limit +-100"], "#2196F3"),
        (500, "rightPID", ["Kp=3.3 Ki=0.30 Kd=0.05", "Limit +-100"], "#2196F3")]:
        mkrect(s, x, 300, 130, 75, fill="#E3F2FD", stroke=col)
        mklines(s, x+65, 318, [name] + params, size=11, fill=col)
    mkarrow(s, 380, 206, 215, 300, "#2196F3")
    mkarrow(s, 480, 206, 565, 300, "#2196F3")
    # Encoder feedback
    for x, lb in [(150, "Left Enc Speed"), (500, "Right Enc Speed")]:
        mkrect(s, x, 410, 130, 40, fill="#FFF3E0", stroke="#FF9800")
        mktext(s, x+65, 435, lb, size=11)
        mkarrow(s, x+65, 410, x+65, 375, "#FF9800", sw=1.2)
    # Motor output
    mkrect(s, 290, 410, 200, 40, fill="#E8F5E9", stroke="#4CAF50")
    mktext(s, 390, 435, "PWM -> H-Bridge -> Motors", size=13, bold=True)
    mkarrow(s, 280, 337, 315, 410, "#4CAF50")
    mkarrow(s, 565, 375, 465, 410, "#4CAF50")
    mktext(s, 390, 30, "PID Cascade Control Structure", size=20, bold=True)
    mktext(s, 390, 55, "Outer: steerPID(position) -> Inner: leftPID/rightPID(speed)", size=13, fill="#888")
    save(s, "03_pid_cascade.svg")

# ====================================================================
# 4: MPU6050 Signal Chain (820x260)
# ====================================================================
def draw_mpu():
    s = svg(820, 260)
    steps = [
        ("MPU6050\nZ-Axis ADC", "#9C27B0", "#F3E5F5"),
        ("Spike Detect\n>100dps discard", "#E65100", "#FFF3E0"),
        ("Low-Pass\nFilter a=0.7", "#1565C0", "#E3F2FD"),
        ("Debias\nAdaptive Track", "#2E7D32", "#E8F5E9"),
        ("Deadband\n+-0.15 dps", "#6A1B9A", "#F3E5F5"),
        ("Integration\nSum(w*dt)", "#C62828", "#FFEBEE"),
        ("Yaw Angle\n+-180 deg", "#4CAF50", "#E8F5E9"),
    ]
    for i, (label, st, fill) in enumerate(steps):
        x = 30 + i * 110
        mkrect(s, x, 100, 95, 70, fill=fill, stroke=st)
        mklines(s, x+47, 118, label.split("\n"), size=11, fill=st)
        if i < 6:
            mkarrow(s, x+95, 135, x+110, 135, st, sw=2)
    mktext(s, 410, 20, "MPU6050 Yaw Angle Processing Pipeline", size=20, bold=True)
    mktext(s, 50, 210, "Adaptive Bias Tracking:", size=13, bold=True, anchor="start")
    mktext(s, 50, 232, "When |rate| < 0.8 dps (not actively turning), slowly converge bias estimate (a=0.01) to compensate temperature drift.",
           size=12, fill="#666", anchor="start")
    save(s, "04_mpu6050_chain.svg")

# ====================================================================
# 5: Avoidance State Machine (780x530)
# ====================================================================
def draw_avoid_fsm():
    s = svg(780, 530)
    states = [
        ("AVOID2_IDLE", "Line Follow", "PID_control()", "#E8F5E9", "#4CAF50", 310, 60),
        ("AVOID2_STOP", "Stop & Snapshot", "Record origin_yaw", "#FFF3E0", "#FF9800", 310, 150),
        ("AVOID2_TR", "Turn Right 45", "TurnToAngle(+45)", "#FCE4EC", "#E91E63", 310, 250),
        ("AVOID2_FW", "Go Forward", "PID_ctrl_head(60)", "#E3F2FD", "#2196F3", 540, 250),
        ("AVOID2_TL", "Turn Left 90", "TurnToAngle(-45)", "#FCE4EC", "#E91E63", 540, 370),
        ("AVOID2_SK", "Seek Line", "PID_ctrl_head(60)", "#F3E5F5", "#9C27B0", 310, 370),
    ]
    for name, desc, action, fill, stroke, x, y in states:
        mkrect(s, x, y, 170, 72, fill=fill, stroke=stroke)
        mktext(s, x+85, y+18, name, size=11, bold=True, fill=stroke)
        mktext(s, x+85, y+36, desc, size=13, bold=True)
        mktext(s, x+85, y+56, action, size=10, fill="#888")
    trans = [
        (480, 170, 480, 250, "dist<25cm"), (480, 222, 480, 250, "1 frame"),
        (480, 322, 540, 322, "err<3deg"), (625, 322, 625, 370, "pulse>1350"),
        (540, 442, 395, 442, "err<3deg"), (395, 442, 395, 60, "found/timeout")
    ]
    for x1, y1, x2, y2, lbl in trans:
        mkarrow(s, x1, y1, x2, y2, "#666")
        mx, my = (x1+x2)//2, (y1+y2)//2
        mktext(s, mx+(15 if x1==x2 else 0), my-8, lbl, size=10, fill="#E65100")
    mktext(s, 390, 20, "Obstacle Avoidance State Machine", size=20, bold=True)
    save(s, "05_avoid_fsm.svg")

# ====================================================================
# 6: Corner Turn State Machine (830x530)
# ====================================================================
def draw_corner_fsm():
    s = svg(830, 530)
    states = [
        ("CORNER2_IDLE", "Wait Corner", "PID_control()", "#E8F5E9", "#4CAF50", 320, 50),
        ("CORNER2_ADV", "Advance", "PID_ctrl_head", "#FFF3E0", "#FF9800", 320, 150),
        ("CORNER2_TRN1", "Turn +-137 deg", "TurnToAngle", "#FCE4EC", "#E91E63", 320, 255),
        ("CORNER2_STR", "Diagonal", "PID_ctrl_head", "#E3F2FD", "#2196F3", 540, 255),
        ("CORNER2_ADV2", "Advance2", "PID_ctrl_head", "#FFF3E0", "#FF9800", 540, 380),
        ("CORNER2_TRN2", "Turn Back", "TurnToAngle(yaw0)", "#FCE4EC", "#E91E63", 320, 380),
    ]
    for name, desc, action, fill, stroke, x, y in states:
        mkrect(s, x, y, 175, 78, fill=fill, stroke=stroke)
        mktext(s, x+87, y+18, name, size=11, bold=True, fill=stroke)
        mktext(s, x+87, y+36, desc, size=13, bold=True)
        mktext(s, x+87, y+56, action, size=10, fill="#888")
    trans = [
        (495, 128, 495, 150, "corner+3db"), (495, 228, 495, 255, "pulse>200"),
        (495, 333, 540, 333, "err<3deg"), (627, 333, 627, 380, "line/dist"),
        (540, 458, 407, 458, "pulse>300"), (407, 458, 407, 128, "err<3deg"),
    ]
    for x1, y1, x2, y2, lbl in trans:
        mkarrow(s, x1, y1, x2, y2, "#666")
        mx, my = (x1+x2)//2, (y1+y2)//2
        mktext(s, mx+(15 if x1==x2 else 0), my-8, lbl, size=10, fill="#E65100")
    mktext(s, 415, 20, "Corner Turn Diagonal Navigation State Machine", size=20, bold=True)
    save(s, "06_corner_fsm.svg")

# ====================================================================
# 7: Parameters Table (820x620)
# ====================================================================
def draw_params_table():
    s = svg(820, 590)
    mktext(s, 410, 28, "Key Parameters Summary", size=20, bold=True)
    mktext(s, 120, 54, "Avoidance Params", size=14, bold=True, anchor="start")

    def draw_table(data, x0, y0, col_ws, header_color):
        y = y0
        for hi, (w, txt) in enumerate(zip(col_ws, data[0])):
            mkrect(s, x0+sum(col_ws[:hi]), y, w, 30, rx=0, fill=header_color, stroke=header_color)
            mktext(s, x0+sum(col_ws[:hi])+w/2, y+20, txt, size=12, fill="#fff", bold=True)
        y += 30
        for row in data[1:]:
            for i in range(len(col_ws)):
                x = x0 + sum(col_ws[:i])
                mkrect(s, x, y, col_ws[i], 28, rx=0, fill="#FFF8E1" if i==0 else "#fff", stroke="#ddd")
                mktext(s, x+col_ws[i]/2, y+19, row[i], size=11)
            y += 28

    avoid_data = [
        ["Parameter", "Value", "Description"],
        ["AVOID_TURN_DEG", "45 deg", "Right turn evade angle"],
        ["AVOID_RETURN_DEG", "45 deg", "Left turn return angle (net -90)"],
        ["AVOID_DIST_PULSES", "1350", "Forward distance during avoidance"],
        ["AVOID_SPEED", "60", "Speed during avoidance"],
        ["AVOID_OBSTACLE_CM", "25 cm", "Ultrasonic trigger threshold"],
        ["AVOID_CONVERGE_THRESH", "3.0 deg", "Turn completion threshold"],
        ["AVOID_SEEK_MAX_PULSES", "2500", "Seek-line timeout distance"],
    ]
    draw_table(avoid_data, 20, 72, [210, 100, 380], "#FF9800")

    y_next = 72 + 30 + 8*28 + 8
    mktext(s, 120, y_next, "Corner Turn Params", size=14, bold=True, anchor="start")

    corner_data = [
        ["Parameter", "Value", "Description"],
        ["CORNER_TURN_DEG", "137 deg", "Turn angle (compensated)"],
        ["CORNER_STRAIGHT_PULSES", "6100", "Max straight distance"],
        ["CORNER_STRAIGHT_SPEED", "80", "Straight phase speed"],
        ["CORNER_ADVANCE_PULSES", "200", "Post-corner advance to align"],
        ["CORNER_RETURN_ADVANCE_PULSES", "300", "Post-line advance to align"],
        ["CORNER_CONVERGE_THRESH", "3.0 deg", "Turn completion threshold"],
        ["CORNER_DETECT_DEBOUNCE", "3 frames", "Corner detection debounce"],
    ]
    draw_table(corner_data, 20, y_next+22, [230, 100, 370], "#4CAF50")

    pid_y = y_next + 22 + 30 + 8*28 + 12
    mktext(s, 120, pid_y, "PID Controller Params", size=14, bold=True, anchor="start")

    pid_data = [
        ["Controller", "Kp", "Ki", "Kd", "Out Limit", "Sum Limit", "Purpose"],
        ["steerPID", "0.9", "0.03", "0.20", "+-80", "40", "Line steering"],
        ["leftPID", "3.7", "0.30", "0.05", "+-100", "80", "Left wheel speed"],
        ["rightPID", "3.3", "0.30", "0.05", "+-100", "80", "Right wheel speed"],
        ["anglePID", "1.3", "0.15", "0.50", "+-40", "30", "Heading angle"],
    ]
    pid_ws = [110, 70, 70, 70, 80, 80, 200]
    draw_table(pid_data, 20, pid_y+22, pid_ws, "#2196F3")

    save(s, "07_parameters_table.svg")


if __name__ == "__main__":
    print(f"Generating SVG diagrams to {OUT_DIR}/")
    draw_track()
    draw_system()
    draw_pid()
    draw_mpu()
    draw_avoid_fsm()
    draw_corner_fsm()
    draw_params_table()
    print(f"\nDone! 7 SVG diagrams ready for Word/PPT/PDF embedding.")
