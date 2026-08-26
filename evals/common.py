"""Shared helpers for the eval suite.

Every eval writes runs/<series>/evals/<name>.json shaped
{"rows": [...], "summary": {...}} (the dashboard renders these
generically), plus any CSV/SVG artifacts next to it.
"""

import json
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Light-mode slots from the validated reference palette (dataviz skill).
PALETTE = {
    "series1": "#2a78d6", "series2": "#eb6834", "series3": "#1baf7a",
    "serious": "#e34948", "good": "#008300",
    "ink": "#0b0b0b", "ink2": "#52514e", "grid": "#dddcd8",
    "surface": "#fcfcfb",
}


def series_dir(series):
    return os.path.join(REPO, "runs", series)


def eval_dir(series):
    d = os.path.join(series_dir(series), "evals")
    os.makedirs(d, exist_ok=True)
    return d


def episodes(series):
    """Finished episodes: list of (ep_number, summary_dict, ep_dir)."""
    sdir = series_dir(series)
    out = []
    if not os.path.isdir(sdir):
        return out
    for name in sorted(os.listdir(sdir)):
        if not name.startswith("ep_"):
            continue
        ep_path = os.path.join(sdir, name)
        spath = os.path.join(ep_path, "summary.json")
        if os.path.exists(spath):
            with open(spath) as f:
                out.append((int(name.split("_")[1]), json.load(f),
                            ep_path))
    return out


def load_json(path):
    with open(path) as f:
        return json.load(f)


def write_eval(series, name, rows, summary):
    path = os.path.join(eval_dir(series), f"{name}.json")
    with open(path, "w") as f:
        json.dump({"rows": rows, "summary": summary}, f, indent=2)
    return path


def memory_text(memory_path, cap_bytes=120_000):
    """Concatenate a /memory tree into one prompt-ready text blob."""
    chunks = []
    total = 0
    for root, _dirs, files in os.walk(memory_path):
        for name in sorted(files):
            full = os.path.join(root, name)
            rel = os.path.relpath(full, memory_path)
            try:
                with open(full, errors="replace") as f:
                    content = f.read(cap_bytes - total)
            except OSError:
                continue
            chunks.append(f"===== /memory/{rel} =====\n{content}\n")
            total += len(content)
            if total >= cap_bytes:
                chunks.append("===== (memory truncated) =====\n")
                return "".join(chunks)
    if not chunks:
        return "(the /memory directory is empty)"
    return "".join(chunks)


def svg_line_chart(series_points, path, title="", y_label="",
                   w=640, h=240, colors=None, dnf_marks=None):
    """Minimal standalone SVG line chart (one or more named series).

    series_points: {"name": [(x, y), ...]}; y may not be None here —
    filter first.  dnf_marks: [(x, label)] drawn at the top.
    """
    P = PALETTE
    colors = colors or [P["series1"], P["series2"], P["series3"]]
    pad_l, pad_r, pad_t, pad_b = 52, 14, 30, 30
    all_pts = [p for pts in series_points.values() for p in pts]
    xs = [p[0] for p in all_pts] or [0, 1]
    ys = [p[1] for p in all_pts] or [1]
    xmin, xmax = min(xs), max(xs)
    ymax = max(ys) * 1.15 or 1
    def SX(x):
        if xmax == xmin:
            return pad_l + (w - pad_l - pad_r) / 2
        return pad_l + (x - xmin) / (xmax - xmin) * (w - pad_l - pad_r)
    def SY(y):
        return h - pad_b - (y / ymax) * (h - pad_t - pad_b)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" '
             f'viewBox="0 0 {w} {h}" font-family="sans-serif">',
             f'<rect width="{w}" height="{h}" fill="{P["surface"]}"/>',
             f'<text x="{pad_l}" y="18" font-size="13" '
             f'fill="{P["ink"]}" font-weight="bold">{title}</text>']
    for i in range(4):
        yv = ymax / 3 * i
        parts.append(f'<line x1="{pad_l}" x2="{w - pad_r}" '
                     f'y1="{SY(yv):.1f}" y2="{SY(yv):.1f}" '
                     f'stroke="{P["grid"]}" stroke-width="1"/>')
        parts.append(f'<text x="{pad_l - 6}" y="{SY(yv) + 4:.1f}" '
                     f'font-size="10" fill="{P["ink2"]}" '
                     f'text-anchor="end">{yv:.0f}</text>')
    parts.append(f'<text x="14" y="{h / 2:.0f}" font-size="10" '
                 f'fill="{P["ink2"]}" transform="rotate(-90 14 {h / 2})" '
                 f'text-anchor="middle">{y_label}</text>')
    multi = len(series_points) > 1
    for i, (name, pts) in enumerate(series_points.items()):
        color = colors[i % len(colors)]
        if len(pts) > 1:
            d = "M" + " L".join(f"{SX(x):.1f},{SY(y):.1f}"
                                for x, y in pts)
            parts.append(f'<path d="{d}" fill="none" stroke="{color}" '
                         f'stroke-width="2"/>')
        for x, y in pts:
            parts.append(f'<circle cx="{SX(x):.1f}" cy="{SY(y):.1f}" '
                         f'r="4" fill="{color}" '
                         f'stroke="{P["surface"]}" stroke-width="1.5"/>')
        if multi and pts:
            lx, ly = pts[-1]
            parts.append(f'<text x="{SX(lx) + 8:.1f}" y="{SY(ly):.1f}" '
                         f'font-size="10" fill="{P["ink2"]}">{name}</text>')
    for x, label in (dnf_marks or []):
        parts.append(f'<circle cx="{SX(x):.1f}" cy="{SY(ymax * 0.9):.1f}" '
                     f'r="4" fill="none" stroke="{P["serious"]}" '
                     f'stroke-width="2"/>')
        parts.append(f'<text x="{SX(x):.1f}" y="{SY(ymax * 0.9) - 8:.1f}" '
                     f'font-size="9" fill="{P["serious"]}" '
                     f'text-anchor="middle">{label}</text>')
    xs_seen = sorted({p[0] for p in all_pts} |
                     {x for x, _ in (dnf_marks or [])})
    for x in xs_seen:
        parts.append(f'<text x="{SX(x):.1f}" y="{h - 10}" font-size="10" '
                     f'fill="{P["ink2"]}" text-anchor="middle">'
                     f'{x:g}</text>')
    parts.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(parts))
    return path
