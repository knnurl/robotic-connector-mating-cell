# Copied verbatim from tools/gui/mating_panel.py (9401a6c) when it was parked.

# Dark-surface chart chrome + status palette (validated set; see repo docs).
THEME = {
    'page': '#0d0d0d',        # window plane
    'surface': '#1a1a19',     # cards / chart surface
    'ink': '#ffffff',         # primary text
    'ink2': '#c3c2b7',        # secondary text
    'muted': '#898781',       # axis labels, captions
    'grid': '#2c2c2a',        # hairline gridlines / card border
    'baseline': '#383835',    # axis baseline
    'series': '#3987e5',      # plot line (single series per plot)
    'good': '#0ca30c',
    'warning': '#fab219',
    'serious': '#ec835a',
    'critical': '#d03b3b',
}


def _hex_to_rgb(h):
    h = h.lstrip('#')
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def text_on(color):
    """Black or white ink, whichever reads on the given fill."""
    r, g, b = _hex_to_rgb(color)
    return '#0b0b0b' if (0.299 * r + 0.587 * g + 0.114 * b) > 150 else '#ffffff'


def mix(color, other, t):
    """Linear blend color->other by t (for hover shades)."""
    a, b = _hex_to_rgb(color), _hex_to_rgb(other)
    return '#%02x%02x%02x' % tuple(int(round(a[i] + (b[i] - a[i]) * t))
                                   for i in range(3))


def rounded_rect(canvas, x0, y0, x1, y1, r, **kw):
    pts = [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1,
           x1 - r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0]
    return canvas.create_polygon(pts, smooth=True, **kw)
