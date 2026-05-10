"""
Visualiseur 3D avion - Banc de test avionique
Modèle low-poly avec faces pleines + éclairage simple.

Dépendances :
    pip install pyserial pygame PyOpenGL PyOpenGL_accelerate numpy

Usage :
    python visualiseur_avion.py
    python visualiseur_avion.py --port /dev/cu.usbmodem1234 --baud 115200
"""

import sys
import re
import threading
import argparse
import math

import serial
import serial.tools.list_ports

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *


# ─────────────────────────────────────────────
#  Données partagées
# ─────────────────────────────────────────────
class FlightData:
    def __init__(self):
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        self.lock  = threading.Lock()

flight = FlightData()


# ─────────────────────────────────────────────
#  Thread UART
# ─────────────────────────────────────────────
PATTERN = re.compile(
    r"Roll:\s*([-\d.]+).*?Pitch:\s*([-\d.]+).*?Yaw:\s*([-\d.]+)",
    re.IGNORECASE
)

def uart_reader(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[UART] Connecté sur {port} à {baud} baud")
    except serial.SerialException as e:
        print(f"[UART] Erreur : {e}"); return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            m = PATTERN.search(line)
            if m:
                with flight.lock:
                    flight.roll  = float(m.group(1))
                    flight.pitch = float(m.group(2))
                    flight.yaw   = float(m.group(3))
        except Exception:
            pass


# ─────────────────────────────────────────────
#  Helpers géométrie
# ─────────────────────────────────────────────
def normal(p1, p2, p3):
    """Calcule la normale d'un triangle."""
    ax, ay, az = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
    bx, by, bz = p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]
    nx = ay*bz - az*by
    ny = az*bx - ax*bz
    nz = ax*by - ay*bx
    l  = math.sqrt(nx*nx + ny*ny + nz*nz) or 1
    return (nx/l, ny/l, nz/l)

def quad(p1, p2, p3, p4):
    n = normal(p1, p2, p3)
    glNormal3f(*n)
    glVertex3f(*p1); glVertex3f(*p2)
    glVertex3f(*p3); glVertex3f(*p4)

def tri(p1, p2, p3):
    n = normal(p1, p2, p3)
    glNormal3f(*n)
    glVertex3f(*p1); glVertex3f(*p2); glVertex3f(*p3)


# ─────────────────────────────────────────────
#  Modèle avion low-poly
#  Convention : X=droite, Y=haut, Z=avant (nez vers +Z)
# ─────────────────────────────────────────────

def draw_fuselage():
    """
    Fuselage : section octogonale extrudée.
    On définit des anneaux le long de Z et on relie les faces.
    """
    # Profil octogonal à différentes stations Z
    # (z, rayon_h, rayon_v)  — ellipse aplatie
    stations = [
        (-2.8, 0.00, 0.00),   # pointe queue
        (-2.2, 0.18, 0.14),
        (-1.5, 0.22, 0.18),
        (-0.5, 0.24, 0.20),
        ( 0.5, 0.24, 0.20),
        ( 1.2, 0.22, 0.18),
        ( 1.8, 0.16, 0.14),
        ( 2.2, 0.10, 0.09),
        ( 2.8, 0.03, 0.03),   # nez
    ]
    N = 8  # segments octogonaux

    def ring(z, rh, rv):
        pts = []
        for i in range(N):
            a = 2 * math.pi * i / N
            pts.append((rh * math.cos(a), rv * math.sin(a), z))
        return pts

    rings = [ring(z, rh, rv) for z, rh, rv in stations]

    glColor3f(0.25, 0.45, 0.75)  # bleu acier
    glBegin(GL_QUADS)
    for k in range(len(rings)-1):
        r0, r1 = rings[k], rings[k+1]
        for i in range(N):
            j = (i+1) % N
            quad(r0[i], r0[j], r1[j], r1[i])
    glEnd()

    # Bouchons (nez et queue) en triangle fan
    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0, 0, 1)
    glVertex3f(0, 0, stations[-1][0])
    for p in rings[-1] + [rings[-1][0]]:
        glVertex3f(*p)
    glEnd()

    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0, 0, -1)
    glVertex3f(0, 0, stations[0][0])
    for p in reversed(rings[0] + [rings[0][0]]):
        glVertex3f(*p)
    glEnd()


def draw_wing(side=1):
    """
    Aile : profil trapézoïdal avec épaisseur.
    side = 1 (droite) ou -1 (gauche)
    """
    # Points de l'aile vue de dessus (bord d'attaque, bord de fuite)
    # (x, z_leading_edge, z_trailing_edge, épaisseur)
    s = side
    # Enracinement
    xr = s * 0.24
    zle_r, zte_r = 0.6, -0.4   # leading/trailing edge à la racine
    # Bout d'aile
    xw = s * 2.6
    zle_w, zte_w = 0.0, -1.0
    th = 0.06  # demi-épaisseur

    # 8 sommets (4 dessus, 4 dessous)
    tld = (xr, +th, zle_r)  # top leading root
    trd = (xr, +th, zte_r)  # top trailing root
    tlw = (xw, +th*0.4, zle_w)
    trw = (xw, +th*0.4, zte_w)
    bld = (xr, -th, zle_r)
    brd = (xr, -th, zte_r)
    blw = (xw, -th*0.4, zle_w)
    brw = (xw, -th*0.4, zte_w)

    glColor3f(0.20, 0.38, 0.68)
    glBegin(GL_QUADS)
    # Dessus
    quad(tld, tlw, trw, trd)
    # Dessous
    quad(brd, brw, blw, bld)
    # Bord d'attaque
    quad(tld, tlw, blw, bld)
    # Bord de fuite
    quad(trd, brd, brw, trw)
    glEnd()
    # Bout d'aile
    glBegin(GL_QUADS)
    quad(tlw, trw, brw, blw)
    glEnd()
    # Jonction racine (triangle pour boucher proprement)
    glBegin(GL_QUADS)
    quad(tld, trd, brd, bld)
    glEnd()


def draw_vertical_stabilizer():
    """Dérive verticale trapézoïdale."""
    # Base sur le fuselage, sommet vers le haut
    th = 0.04
    # Racine
    zr_le, zr_te = -1.8, -2.6
    yr = 0.18
    # Sommet
    zs_le, zs_te = -1.9, -2.4
    ys = 0.70

    rl  = ( th, yr, zr_le)
    rr  = (-th, yr, zr_le)
    rt  = ( th, yr, zr_te)
    rb  = (-th, yr, zr_te)
    sl  = ( th, ys, zs_le)
    sr  = (-th, ys, zs_le)
    st  = ( th, ys, zs_te)
    sb  = (-th, ys, zs_te)

    glColor3f(0.22, 0.42, 0.72)
    glBegin(GL_QUADS)
    quad(rl, sl, st, rt)   # côté droit
    quad(rr, rb, sb, sr)   # côté gauche
    quad(rl, rr, sr, sl)   # bord d'attaque
    quad(rt, st, sb, rb)   # bord de fuite
    quad(sl, sr, sb, st)   # sommet
    glEnd()


def draw_horizontal_stabilizer(side=1):
    """Stabilisateur horizontal (petit, arrière)."""
    s = side
    th = 0.035
    xr = s * 0.18
    zle_r, zte_r = -1.9, -2.5
    xw = s * 1.1
    zle_w, zte_w = -2.0, -2.6

    tld = (xr, +th, zle_r)
    trd = (xr, +th, zte_r)
    tlw = (xw, +th*0.5, zle_w)
    trw = (xw, +th*0.5, zte_w)
    bld = (xr, -th, zle_r)
    brd = (xr, -th, zte_r)
    blw = (xw, -th*0.5, zle_w)
    brw = (xw, -th*0.5, zte_w)

    glColor3f(0.20, 0.38, 0.68)
    glBegin(GL_QUADS)
    quad(tld, tlw, trw, trd)
    quad(brd, brw, blw, bld)
    quad(tld, tlw, blw, bld)
    quad(trd, brd, brw, trw)
    quad(tlw, trw, brw, blw)
    quad(tld, trd, brd, bld)
    glEnd()


def draw_engine(side=1):
    """Réacteur cylindrique sous l'aile."""
    s  = side
    x  = s * 1.4
    y  = -0.18
    z0 = -0.2   # arrière
    z1 =  0.5   # avant
    r  = 0.10
    N  = 10

    def cring(z):
        return [(x + r*math.cos(2*math.pi*i/N),
                 y + r*math.sin(2*math.pi*i/N),
                 z) for i in range(N)]

    ra, rf = cring(z0), cring(z1)

    glColor3f(0.18, 0.18, 0.22)
    glBegin(GL_QUADS)
    for i in range(N):
        j = (i+1) % N
        quad(ra[i], ra[j], rf[j], rf[i])
    glEnd()

    # Nacelle avant (entrée d'air)
    glColor3f(0.12, 0.12, 0.15)
    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0, 0, 1)
    glVertex3f(x, y, z1)
    for p in rf + [rf[0]]:
        glVertex3f(*p)
    glEnd()


def draw_cockpit():
    """Verrière simplifiée — dôme aplati sur le fuselage avant."""
    glColor3f(0.4, 0.7, 0.9)  # bleu clair translucide
    NL, NA = 5, 8
    r_base = 0.13
    z_center = 1.9
    y_center = 0.10
    height = 0.10

    def dome_pt(il, ia):
        phi = math.pi/2 * il / NL        # 0 → pi/2
        theta = math.pi * ia / (NA-1) - math.pi/2
        x = r_base * math.cos(phi) * math.cos(theta)
        y = y_center + height * math.sin(phi)
        z = z_center + r_base * math.cos(phi) * math.sin(theta) * 0.6
        return (x, y, z)

    glBegin(GL_QUADS)
    for il in range(NL-1):
        for ia in range(NA-1):
            p1 = dome_pt(il,   ia)
            p2 = dome_pt(il,   ia+1)
            p3 = dome_pt(il+1, ia+1)
            p4 = dome_pt(il+1, ia)
            quad(p1, p2, p3, p4)
    glEnd()


def draw_plane():
    draw_fuselage()
    draw_wing(side= 1)
    draw_wing(side=-1)
    draw_vertical_stabilizer()
    draw_horizontal_stabilizer(side= 1)
    draw_horizontal_stabilizer(side=-1)
    draw_engine(side= 1)
    draw_engine(side=-1)
    draw_cockpit()


# ─────────────────────────────────────────────
#  Grille de sol
# ─────────────────────────────────────────────
def draw_grid():
    glDisable(GL_LIGHTING)
    glColor3f(0.10, 0.10, 0.13)
    glLineWidth(1.0)
    glBegin(GL_LINES)
    for i in range(-8, 9):
        glVertex3f(i, -1.5, -8); glVertex3f(i, -1.5,  8)
        glVertex3f(-8, -1.5, i); glVertex3f( 8, -1.5, i)
    glEnd()
    glEnable(GL_LIGHTING)


# ─────────────────────────────────────────────
#  HUD via texture pygame
# ─────────────────────────────────────────────
def blit_hud(hud_surface, W, H):
    flipped  = pygame.transform.flip(hud_surface, False, True)
    tex_data = pygame.image.tostring(flipped, "RGBA", False)

    tid = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tid)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W, H, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, tex_data)

    glMatrixMode(GL_PROJECTION)
    glPushMatrix(); glLoadIdentity()
    glOrtho(0, W, 0, H, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix(); glLoadIdentity()

    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_TEXTURE_2D)
    glColor4f(1,1,1,1)
    glBegin(GL_QUADS)
    glTexCoord2f(0,0); glVertex2f(0,0)
    glTexCoord2f(1,0); glVertex2f(W,0)
    glTexCoord2f(1,1); glVertex2f(W,H)
    glTexCoord2f(0,1); glVertex2f(0,H)
    glEnd()
    glDisable(GL_TEXTURE_2D)
    glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)

    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glDeleteTextures([tid])


# ─────────────────────────────────────────────
#  Détection port
# ─────────────────────────────────────────────
def auto_detect_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ["stm","nucleo","usb serial","uart","cp210","ch340","ftdi"]):
            return p.device
    return ports[0].device if ports else None


# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",  default=None)
    parser.add_argument("--baud",  default=115200, type=int)
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    if port is None:
        print("[UART] Aucun port — mode démo")
    else:
        threading.Thread(target=uart_reader, args=(port, args.baud), daemon=True).start()

    pygame.init()
    pygame.font.init()
    W, H = 1000, 650
    pygame.display.set_mode((W, H), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Banc de test avionique — Visualiseur 3D")

    try:
        font    = pygame.font.SysFont("Menlo", 15)
        font_sm = pygame.font.SysFont("Menlo", 13)
    except Exception:
        font    = pygame.font.Font(None, 18)
        font_sm = pygame.font.Font(None, 15)

    # ── OpenGL setup ──
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    glShadeModel(GL_SMOOTH)

    glLightfv(GL_LIGHT0, GL_POSITION,  [3.0, 5.0, 4.0, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE,   [1.0, 1.0, 1.0, 1.0])
    glLightfv(GL_LIGHT0, GL_AMBIENT,   [0.25, 0.25, 0.30, 1.0])
    glLightfv(GL_LIGHT0, GL_SPECULAR,  [0.5, 0.5, 0.5, 1.0])

    glEnable(GL_LIGHT1)
    glLightfv(GL_LIGHT1, GL_POSITION, [-4.0, -2.0, 2.0, 1.0])
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  [0.2, 0.25, 0.35, 1.0])
    glLightfv(GL_LIGHT1, GL_AMBIENT,  [0.0, 0.0, 0.0, 1.0])

    glClearColor(0.04, 0.04, 0.07, 1.0)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, W/H, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

    clock = pygame.time.Clock()
    angle_demo = 0.0  # rotation lente en mode démo

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit(); sys.exit()
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                pygame.quit(); sys.exit()

        with flight.lock:
            roll  = flight.roll
            pitch = flight.pitch
            yaw   = flight.yaw

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluLookAt(0, 2.5, 7,   0, 0, 0,   0, 1, 0)

        draw_grid()

        glPushMatrix()
        glRotatef(yaw,   0, 1, 0)
        glRotatef(pitch, 1, 0, 0)
        glRotatef(roll,  0, 0, 1)
        draw_plane()
        glPopMatrix()

        # ── HUD ──
        hud = pygame.Surface((W, H), pygame.SRCALPHA)
        hud.fill((0,0,0,0))

        pygame.draw.rect(hud, (0,0,0,160), pygame.Rect(10,10,230,95), border_radius=6)

        def t(txt, x, y, color, f=font):
            hud.blit(f.render(txt, True, color), (x, y))

        t("BANC DE TEST AVIONIQUE", 18, 17, (80,180,255))
        t(f"ROLL  : {roll:+8.2f} deg", 18, 38, (160,255,160))
        t(f"PITCH : {pitch:+8.2f} deg", 18, 56, (160,255,160))
        t(f"YAW   : {yaw:+8.2f} deg",  18, 74, (160,255,160))

        port_txt   = f"PORT: {port}" if port else "PORT: non connecte"
        port_color = (80,200,80) if port else (220,80,80)
        t(port_txt, W - font_sm.size(port_txt)[0] - 12, 14, port_color, font_sm)
        t("ESC pour quitter", W - font_sm.size("ESC pour quitter")[0] - 12, H-22, (60,60,60), font_sm)

        blit_hud(hud, W, H)

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()