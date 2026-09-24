"""The panel's look: colour tokens, the three text sizes, and the Qt style sheet.

ISA-101: a muted grey base where healthy and idle things are quiet, so the
few saturated colours mean something. IEC 60073: each hue has one meaning -
red danger/emergency (only STOP NOW and real faults), amber abnormal, green
a process running healthy, blue the one next mandatory step, grey neutral,
done or blocked. Red and green never carry meaning alone: every indicator
pairs its colour with a shape and a word (logic.NORMAL/WARN/FAULT/ACTIVE).

Not the Tk panel's palette (theme.py, retired with it): that had an orange
'serious' level, which IEC 60073 has no meaning for, and blue plot lines.
"""

T = {
    'page': '#1d1f21',          # dark neutral grey, not black
    'surface': '#25282b',
    'raised': '#30343a',        # neutral buttons
    'raised_hi': '#3a3f46',
    'sunken': '#191a1c',        # inputs, plot wells
    'line': '#3b3f45',
    'line_dim': '#2d3034',
    'ink': '#ecebe7',           # live values - the brightest text
    'ink2': '#b9b8b2',          # body
    'muted': '#8a8b85',         # section headers, captions, units
    'off_ink': '#62645f',       # blocked controls
    'off_bg': '#222427',
    # reserved hues - one meaning each
    'red': '#cf3a31',
    'amber': '#e2a41f',
    'green': '#44a262',
    'blue': '#2f6bd1',
}
INK_ON = {'red': '#ffffff', 'amber': '#17181a', 'green': '#0f1a12', 'blue': '#ffffff'}

UI_FONT = '"Lato", "DejaVu Sans", sans-serif'
MONO_FONT = 'DejaVu Sans Mono'
# Three sizes only: L = banner title and live values, M = body and buttons,
# S = section headers, captions and units.
L, M, S = 20, 13, 11

# indicator level -> (colour token, shape)
LEVEL = {'normal': ('muted', 'circle'), 'active': ('green', 'circle'),
         'warn': ('amber', 'triangle'), 'fault': ('red', 'square')}

QSS = f"""
* {{ font-family: {UI_FONT}; font-size: {M}px; }}
QMainWindow, QWidget#central {{ background: {T['page']}; }}
QWidget {{ color: {T['ink2']}; }}
QFrame#panel {{ background: {T['surface']}; border: 1px solid {T['line_dim']};
               border-radius: 6px; }}
QFrame#panel[inactive="true"] {{ background: {T['page']}; border: 1px dashed {T['line']}; }}
QFrame#bar {{ background: {T['surface']}; border-bottom: 1px solid {T['line_dim']}; }}
QFrame#stopbar {{ background: {T['surface']}; border-top: 1px solid {T['line']}; }}
QLabel {{ background: transparent; }}
QLabel[role="section"] {{ color: {T['muted']}; font-size: {S}px; font-weight: bold; }}
QLabel[role="caption"], QLabel[role="unit"] {{ color: {T['muted']}; font-size: {S}px; }}
QLabel[role="value"] {{ color: {T['ink']}; font-family: "{MONO_FONT}"; font-size: {L}px;
                       font-weight: bold; }}
QLabel[role="value"][level="warn"] {{ color: {T['amber']}; }}
QLabel[role="value"][level="off"] {{ color: {T['off_ink']}; }}
QLabel[role="mono"] {{ color: {T['ink2']}; font-family: "{MONO_FONT}"; font-size: {M}px; }}
QLabel[role="mode"] {{ color: {T['ink']}; font-weight: bold; }}

QFrame#banner {{ border-radius: 6px; background: {T['surface']};
                border: 1px solid {T['line']}; }}
QFrame#banner[level="warn"] {{ background: {T['amber']}; border: none; }}
QFrame#banner[level="fault"] {{ background: {T['red']}; border: none; }}
QLabel#bannerTitle {{ font-size: {L}px; font-weight: bold; color: {T['ink']}; }}
QLabel#bannerDetail {{ color: {T['ink2']}; }}
QLabel#bannerTitle[level="warn"], QLabel#bannerDetail[level="warn"] {{ color: {INK_ON['amber']}; }}
QLabel#bannerTitle[level="fault"], QLabel#bannerDetail[level="fault"] {{ color: {INK_ON['red']}; }}

QPushButton {{ background: {T['raised']}; color: {T['ink']}; border: 1px solid {T['line']};
              border-radius: 5px; padding: 7px 12px; font-weight: bold; }}
QPushButton:hover {{ background: {T['raised_hi']}; }}
QPushButton:pressed {{ background: {T['sunken']}; }}
QPushButton[state="blocked"] {{ background: {T['off_bg']}; color: {T['off_ink']};
                               border: 1px solid {T['line_dim']}; }}
QPushButton[state="next"] {{ background: {T['blue']}; color: {INK_ON['blue']};
                            border: 1px solid {T['blue']}; }}
QPushButton[state="next"]:hover {{ background: #3a78e0; }}
QPushButton[state="pending"] {{ background: {T['sunken']}; color: {T['ink']};
                               border: 1px dashed {T['ink2']}; }}
QPushButton[state="confirm"] {{ background: {T['raised']}; color: {T['amber']};
                               border: 1px solid {T['amber']}; }}
QPushButton[kind="quiet"] {{ font-weight: normal; padding: 4px 8px; }}
QPushButton#stopNow {{ background: {T['red']}; color: {INK_ON['red']}; border: none;
                      font-size: {L}px; font-weight: bold; border-radius: 6px; }}
QPushButton#stopNow:hover {{ background: #dc4a40; }}
QPushButton#stopNow[state="pending"] {{ background: #8f2821; }}
QPushButton#bannerAction {{ background: rgba(0,0,0,0.18); border: 1px solid rgba(0,0,0,0.35);
                           color: inherit; }}
QPushButton#bannerAction[state="next"] {{ background: {T['blue']}; color: #ffffff;
                                         border: 1px solid #ffffff; }}

QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {{
    background: {T['sunken']}; color: {T['ink']}; border: 1px solid {T['line']};
    border-radius: 4px; padding: 3px 6px; font-family: "{MONO_FONT}"; }}
QComboBox:disabled, QLineEdit:disabled {{ color: {T['off_ink']}; }}
QComboBox QAbstractItemView {{ background: {T['surface']}; color: {T['ink']};
                              selection-background-color: {T['raised_hi']}; }}
QLineEdit[pending="true"] {{ border: 1px solid {T['amber']}; }}
QCheckBox {{ color: {T['ink2']}; }}
QPlainTextEdit {{ background: {T['sunken']}; color: {T['ink2']}; border: 1px solid {T['line_dim']};
                 font-family: "{MONO_FONT}"; font-size: {S}px; }}
QSlider::groove:horizontal {{ height: 4px; background: {T['line']}; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {T['ink2']}; border-radius: 2px; }}
QSlider::handle:horizontal {{ background: {T['ink']}; width: 14px; margin: -6px 0;
                             border-radius: 7px; }}
QSlider::handle:horizontal:disabled {{ background: {T['off_ink']}; }}
QSlider::sub-page:horizontal:disabled {{ background: {T['off_ink']}; }}
QScrollArea {{ border: none; background: {T['page']}; }}
QScrollArea > QWidget > QWidget {{ background: {T['page']}; }}
QToolTip {{ background: {T['surface']}; color: {T['ink']}; border: 1px solid {T['line']};
           padding: 4px; }}
"""
