#!/usr/bin/env python3
"""Summarise an FR3 cell log.

    python3 tools/fr3/analyse_trace.py                  # newest log
    python3 tools/fr3/analyse_trace.py <log.jsonl> ...

Two kinds, both one JSON object per line:

  cell_*.jsonl      cell_panel's session trace: header and outcome, ladder
                    events, ALIGN iterations and steps, TRACK as the panel
                    saw it (start, stop, the node's status changes)
  tracking_*.jsonl  tracking_node's per-tick log, one per TRACK run
                    (log_record in mating_controller/src/tracking_node.cpp)

With no argument it takes the newest of either under $FR3_LOG_DIR, day
folders included, else under tools/fr3/logs. Files are streamed: a panel
trace carries the 50 Hz robot samples and can run to hundreds of MB.
"""

import collections
import datetime
import json
import os
import pathlib
import re
import sys
import textwrap

import numpy as np

LOG_DIR = pathlib.Path(__file__).with_name('logs')
PATTERNS = ('cell_*.jsonl', 'tracking_*.jsonl')
# ALIGN step records: commanded field, achieved field, unit
STEPS = {'translate': ('cmd_d_base_mm', 'achieved_trans_mm', 'mm'),
         'level': ('clamped_cmd_deg', 'achieved_rot_deg', 'deg'),
         'inplane': ('cmd_deg', 'achieved_rot_deg', 'deg')}


def newest_log():
    """Newest log by mtime: $FR3_LOG_DIR first, then tools/fr3/logs."""
    env = os.environ.get('FR3_LOG_DIR')
    for root in ([pathlib.Path(env)] if env else []) + [LOG_DIR]:
        found = [f for pat in PATTERNS for f in root.rglob(pat)]
        if found:
            return max(found, key=lambda f: f.stat().st_mtime)
    return None


def records(path):
    """Each JSON object in turn. A crash can leave a torn last line."""
    with open(path) as f:
        for line in f:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def mean(values):
    v = np.array(values, dtype=float)     # None -> nan
    v = v[np.isfinite(v)]
    return v.mean() if v.size else float('nan')


def spread(values, unit):
    v = np.array([x for x in values if x is not None], dtype=float)
    if not v.size:
        return 'none'
    p50, p95 = np.percentile(v, [50, 95])
    return f'p50 {p50:.2f}  p95 {p95:.2f}  max {v.max():.2f} {unit}'


def reason_kind(reason):
    """Numbers out, so 'lead 12.3 mm' and 'lead 14.1 mm' count as one."""
    return re.sub(r'-?\d+(\.\d+)?', '#', reason or '') or '?'


def wrapped(label, text):
    return textwrap.wrap(text, 76, initial_indent=f'  {label:10s} ',
                         subsequent_indent=' ' * 13)


def summarise_tracking(path):
    n = published = clamped = 0
    t0 = t1 = None
    holds = collections.Counter()
    pos, rot, lead_mm, lead_deg, buzz, policies = [], [], [], [], [], []
    for r in records(path):
        n += 1
        t0, t1 = (r['t'] if t0 is None else t0), r['t']
        if r.get('published'):
            published += 1
            clamped += bool(r.get('reason'))        # 'clamped: ...'
        else:
            holds[reason_kind(r.get('reason'))] += 1
        pos.append(r.get('pos_err_mm'))
        rot.append(r.get('rot_err_deg'))
        lead_mm.append(r.get('lead_mm'))
        lead_deg.append(r.get('lead_deg'))
        buzz.append(r.get('buzz_nm'))            # absent before the watchdog
        if not policies or policies[-1][1] != r.get('policy'):
            policies.append((r['t'], r.get('policy')))
    out = [f'=== {path.name}  (tracking_node, {n} ticks) ===']
    if not n:
        return out
    dur, held = t1 - t0, n - published
    out += [f'  duration   {dur:.1f} s',
            f'  tracking   {dur * published / n:.1f} s ({published / n:.0%}),'
            f' {clamped} ticks clamped',
            f'  holding    {dur * held / n:.1f} s ({held / n:.0%})']
    out += [f'    {k:6d}  {kind}' for kind, k in holds.most_common()]
    out += [f'  pos error  {spread(pos, "mm")}',
            f'  rot error  {spread(rot, "deg")}',
            f'  lead       {spread(lead_mm, "mm")}',
            f'             {spread(lead_deg, "deg")}',
            f'  policy     {policies[0][1]}' + ''.join(
                f', {t - t0:.1f} s -> {p}' for t, p in policies[1:]),
            f'  buzz       {spread(buzz, "Nm")}']
    return out


def summarise_cell(path):
    counts = collections.Counter()
    head = end = t0 = t1 = None
    samples, rate_min, force_max = 0, float('inf'), 0.0
    iters, steps = [], collections.defaultdict(list)
    starts, status, policy = [], [], []
    for r in records(path):
        rec, t = r.get('rec'), r.get('t')
        counts[rec] += 1
        t0, t1 = (t if t0 is None else t0), t
        if rec == 'session_start':
            head = r
        elif rec == 'session_end':
            end = r
        elif rec == 'sample':
            samples += 1
            rate_min = min(rate_min, r.get('success_rate', rate_min))
            force_max = max(force_max, float(np.linalg.norm(r['force'])))
        elif rec == 'iter':
            iters.append((r.get('err_mm'), r.get('tilt_deg'),
                          r.get('inplane_err_deg')))
        elif rec in STEPS:
            cmd_k, ach_k, _ = STEPS[rec]
            cmd = r.get(cmd_k)
            if isinstance(cmd, list):                  # a vector, in mm
                cmd = float(np.linalg.norm(cmd))
            elif cmd is not None:                      # signed; achieved is |.|
                cmd = abs(float(cmd))
            steps[rec].append((r.get('ok'), cmd, r.get(ach_k)))
        elif rec == 'track_start':
            starts.append(bool(r.get('ok')))
        elif rec == 'track_status':
            status.append((t, r.get('state'), r.get('reason')))
        elif rec == 'track_policy':
            policy.append(f"{r.get('policy')}"
                          + ('' if r.get('ok') else ' (refused)'))
    out = [f'=== {path.name}  (cell_panel, {sum(counts.values())} '
           'records) ===']
    if t0 is None:
        return out
    head = head or {}
    when = datetime.datetime.fromtimestamp(t0).strftime('%Y-%m-%d %H:%M:%S')
    out.append(f'  session    {when}, {t1 - t0:.1f} s, '
               f'tab {head.get("tab", "?")}')
    out += wrapped('header', '  '.join(
        f'{k} {v}' for k, v in head.items()
        if k not in ('rec', 't', 'tab') and not isinstance(v, list)))
    out.append('  outcome    ' + (str(end.get('outcome') or '-') if end
                                  else 'no session_end - crashed, or open'))
    out += wrapped('events', ', '.join(f'{k} {v}' for k, v in counts.items()
                                       if k != 'sample'))
    if samples:
        out.append(f'  samples    {samples}, control success min '
                   f'{rate_min * 100:.1f}%, |F ext| max {force_max:.1f} N')
    if iters:
        out.append(f'-- ALIGN: {len(iters)} iterations --')
        for name, col, unit in (('|e|', 0, 'mm'), ('tilt', 1, 'deg'),
                                ('in-plane', 2, 'deg')):
            v = np.array([it[col] for it in iters], dtype=float)
            if np.isfinite(v).any():
                out.append(f'  {name:9s}  first {v[0]:7.2f}  last {v[-1]:7.2f}'
                           f'  min {np.nanmin(v):7.2f} {unit}')
    for kind, rows in steps.items():
        unit = STEPS[kind][2]
        out.append(f'  {kind:9s}  {len(rows)} steps, '
                   f'{sum(ok is False for ok, _, _ in rows)} failed; mean '
                   f'commanded {mean([c for _, c, _ in rows]):.2f} {unit}, '
                   f'achieved {mean([a for _, _, a in rows]):.2f} {unit}')
    if starts or status or policy:
        out.append('-- TRACK, as the panel saw it --')
        out.append(f'  START      {len(starts)} ({sum(starts)} ok), '
                   f'STOP {counts["track_stop"]}')
        dwell = collections.Counter()
        ends = [s[0] for s in status[1:]] + [t1]
        for (ta, state, _), tb in zip(status, ends):
            dwell[state] += tb - ta
        if dwell:
            out += wrapped('node state', ', '.join(
                f'{s} {d:.1f} s' for s, d in dwell.items()))
        why = collections.Counter(reason_kind(r) for _, _, r in status if r)
        out += [f'    {k:6d}  {kind}' for kind, k in why.most_common()]
        if policy:
            out.append('  policy     ' + ' -> '.join(policy))
    return out


def summarise(path):
    path = pathlib.Path(path)
    if path.name.startswith('tracking_'):
        return summarise_tracking(path)
    return summarise_cell(path)


def main(argv=None):
    paths = sys.argv[1:] if argv is None else argv
    if not paths:
        newest = newest_log()
        if newest is None:
            print('no cell_ or tracking_ logs under $FR3_LOG_DIR or', LOG_DIR)
            return 1
        paths = [newest]
    for p in paths:
        print('\n'.join(summarise(p)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
