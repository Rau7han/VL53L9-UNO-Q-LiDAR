"""Build the ST host postprocessing pipeline; requires a native C compiler."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent

def build():
    transform = ROOT / 'vendor/vl53l9-transform-c/vl53l9-transform-c-lib'
    media = ROOT / 'vendor/media-object'
    out = ROOT / 'build/libvl53l9_transform.so'
    out.parent.mkdir(exist_ok=True)
    includes = [transform/'include/public', transform/'include/private',
                media/'media-c/header-lib/include/public',
                media/'transform-c/header-lib/include/public']
    sources = [ROOT/'transform_bridge.c', *sorted((transform/'src').rglob('*.c'))]
    subprocess.run(['cc', '-std=c11', '-O2', '-shared', '-fPIC',
                    '-DVL53L9_TRANSFORM_LIGHT=0',
                    *[f'-I{p}' for p in includes], *map(str, sources), '-lm', '-o', str(out)], check=True)
    print(out)

if __name__ == '__main__':
    build()
