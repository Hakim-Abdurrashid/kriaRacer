#!/usr/bin/env python3
"""Render the README figures from the interactive explainer page (docs/inside-blaze-racer.html).

Each figure is cropped out of the page with headless Chrome, once per theme, into OUT_DIR/<name>-{light,dark}.png
(README.md shows the one that matches the reader's GitHub theme). The scan view and the lap map show the page's
default frame: the first full-right-lock moment of the lap (4.0 s).
Needs google-chrome and Pillow.  usage: readme_figures.py docs/inside-blaze-racer.html docs/img [figure ...]"""
import pathlib, subprocess, sys, tempfile, urllib.parse
from PIL import Image

FIGS = {  # name: (CSS selector on the page, width in CSS px, elements inside it to hide)
    'pipeline':   ('#hardware .diagram', 900, ''),
    'scan':       ('#seeing .viewbox', 640, ''),
    'lap-map':    ('#driving .lapgrid > figure', 900, 'figcaption'),
    'lap-trace':  ('#driving .lapgrid > .stack', 900, 'p'),
    'speed-rule': ('#ch-map', 640, 'figcaption'),
    'turning':    ('#driving .diagram', 640, ''),
    'cpu':        ('#ch-cpu', 640, 'figcaption'),
}
HEAD = """<script>(function(){var q=new URLSearchParams(location.search);
document.documentElement.setAttribute('data-theme',q.get('theme')||'light');})();</script>\n"""
# after the page has drawn: move the figure to the top of an otherwise empty, transparent page
TAIL = """\n<script>(async function(){
const q=new URLSearchParams(location.search);const sel=q.get('sel'),w=+q.get('w'),hide=(q.get('hide')||'').split('|').filter(Boolean);
if(!sel)return;
await document.fonts.ready;
const vm=document.getElementById('val-map');if(vm)vm.textContent='1 m clear or less \\u2192 2.0 m/s \\u00b7 4 m or more \\u2192 5.0 m/s (\\u00d71.0)';
const t=document.querySelector(sel);hide.forEach(s=>t.querySelectorAll(s).forEach(e=>e.style.display='none'));
const cap=document.createElement('div');cap.style.cssText='width:'+w+'px;padding:0;margin:0;display:grid';
document.body.prepend(cap);cap.appendChild(t);
for(const c of [...document.body.children])if(c!==cap&&c.tagName!=='SCRIPT')c.style.display='none';
document.documentElement.style.background='transparent';document.documentElement.style.colorScheme='normal';
document.body.style.cssText+=';background:transparent;padding:0;margin:0';
})();</script>\n"""


def main():
    page, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    only = set(sys.argv[3:])
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        cap = pathlib.Path(tmp) / 'cap.html'
        cap.write_text(HEAD + page.read_text() + TAIL)
        for name, (sel, w, hide) in FIGS.items():
            if only and name not in only:
                continue
            for theme in ('light', 'dark'):
                png = out / f'{name}-{theme}.png'
                url = cap.as_uri() + '?' + urllib.parse.urlencode({'sel': sel, 'w': w, 'hide': hide, 'theme': theme})
                subprocess.run(['google-chrome', '--headless=new', '--disable-gpu', '--hide-scrollbars',
                                '--force-device-scale-factor=2', '--default-background-color=00000000',
                                f'--window-size={w},1500', '--virtual-time-budget=15000', f'--user-data-dir={tmp}/profile',
                                f'--screenshot={png}', url], check=True, capture_output=True, timeout=120)
                im = Image.open(png).convert('RGBA')
                im = im.crop(im.getchannel('A').getbbox())  # trim the transparent page around the figure
                im.save(png, optimize=True)
                print(f'{png}  {im.size[0]}x{im.size[1]}')


if __name__ == '__main__':
    main()
