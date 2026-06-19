"""Annotate the wiring photos used in the README setup section.

Run from the repo root:  python3 docs/images/annotate.py
Produces the *-annotated.png files referenced by README.md.
"""
import math
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = "/System/Library/Fonts/Supplemental/Arial Bold.ttf"

# Color per logical wire/role so the same role reads the same across both photos.
DATA = (0, 150, 255)     # CAN data cable (adapter <-> board)
POWER = (255, 60, 60)    # power in (24V / XT60)
USB = (255, 180, 0)      # USB-C to computer
MOTOR = (40, 200, 120)   # cable to the motors


def font(size):
    return ImageFont.truetype(FONT_PATH, size)


def draw_arrow(draw, start, end, color, width=7):
    draw.line([start, end], fill=color, width=width)
    ang = math.atan2(end[1] - start[1], end[0] - start[0])
    size = 26
    for da in (math.radians(28), math.radians(-28)):
        x = end[0] - size * math.cos(ang - da)
        y = end[1] - size * math.sin(ang - da)
        draw.line([end, (x, y)], fill=color, width=width)


def label(draw, xy, text, color, fnt, anchor="lt"):
    pad = 12
    # Multi-line support
    lines = text.split("\n")
    widths, heights = [], []
    for ln in lines:
        l, t, r, b = draw.textbbox((0, 0), ln, font=fnt)
        widths.append(r - l)
        heights.append(b - t)
    tw = max(widths)
    line_h = max(heights) + 8
    th = line_h * len(lines)
    x, y = xy
    if "r" in anchor:
        x -= tw + 2 * pad
    if "b" in anchor:
        y -= th + 2 * pad
    box = [x, y, x + tw + 2 * pad, y + th + 2 * pad]
    draw.rectangle(box, fill=(0, 0, 0), outline=color, width=4)
    cy = y + pad
    for ln in lines:
        draw.text((x + pad, cy), ln, fill=(255, 255, 255), font=fnt)
        cy += line_h
    return box


def anchor_point(box, side):
    x0, y0, x1, y1 = box
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    return {
        "l": (x0, cy), "r": (x1, cy),
        "t": (cx, y0), "b": (cx, y1),
    }[side]


def annotate_adapter():
    img = Image.open("docs/images/usb-can-adapter-raw.png").convert("RGB")
    d = ImageDraw.Draw(img)
    f = font(30)

    # Two CAN data cables out the top
    b = label(d, (40, 40), "CAN data cables to the\npower board (2 cables,\none per arm)", DATA, f, "lt")
    draw_arrow(d, anchor_point(b, "r"), (360, 250), DATA)

    # The grey adapter box
    b = label(d, (748, 360), "DM-USB2FDCAN\nUSB-to-CAN\nadapter", (220, 220, 220), f, "rt")
    draw_arrow(d, anchor_point(b, "l"), (455, 470), (220, 220, 220))

    # USB-C cable to computer
    b = label(d, (40, 800), "USB-C to the computer\n(data only)", USB, f, "lt")
    draw_arrow(d, anchor_point(b, "r"), (395, 760), USB)

    img.save("docs/images/usb-can-adapter-annotated.png")


def annotate_board():
    img = Image.open("docs/images/power-distribution-board-raw.png").convert("RGB")
    d = ImageDraw.Draw(img)
    f = font(30)

    # Top thick bundle -> motors
    b = label(d, (760, 50), "To the motors (already\nplugged into the motor;\nplug this end into the board)", MOTOR, f, "rt")
    draw_arrow(d, anchor_point(b, "l"), (360, 330), MOTOR)

    # Left skinny white connector -> CAN data from adapter
    b = label(d, (15, 700), "CAN data from the\nUSB adapter (same\ncable as the photo\nabove)", DATA, f, "lt")
    draw_arrow(d, anchor_point(b, "t"), (300, 628), DATA)

    # Yellow XT60 -> power in
    b = label(d, (470, 720), "Power in (24V) —\nplugs into the\nyellow connector", POWER, f, "lt")
    draw_arrow(d, anchor_point(b, "l"), (415, 615), POWER)

    img.save("docs/images/power-distribution-board-annotated.png")


if __name__ == "__main__":
    annotate_adapter()
    annotate_board()
    print("wrote annotated images")
