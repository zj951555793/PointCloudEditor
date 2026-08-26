#!/usr/bin/env python3
import json
from collections import defaultdict
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / 'examples' / 'qt_editor' / 'config' / 'camera_models.json'
errors = []
try:
    data = json.loads(PATH.read_text(encoding='utf-8'))
except Exception as exc:
    print(f'{PATH}: invalid JSON: {exc}')
    sys.exit(1)


def norm_usb(value):
    text = str(value or '').strip().upper()
    if text.startswith('0X'):
        text = text[2:]
    if not text:
        return ''
    return text.zfill(4)[-4:]

profiles = data.get('cameras')
by_model = defaultdict(list)
seen = set()
if not isinstance(profiles, list) or not profiles:
    errors.append('cameras must be a non-empty array')
else:
    for i, profile in enumerate(profiles):
        if not isinstance(profile, dict):
            errors.append(f'cameras[{i}] must be an object')
            continue
        model = str(profile.get('model', '')).strip().upper()
        vid = norm_usb(profile.get('vid'))
        pid = norm_usb(profile.get('pid'))
        if not model:
            errors.append(f'cameras[{i}].model is required')
        if not vid:
            errors.append(f'{model or i}: vid is required')
        if not pid:
            errors.append(f'{model or i}: pid is required')
        key = (model, vid, pid)
        if key in seen:
            errors.append(f'duplicate camera profile: model={model} vid={vid} pid={pid}')
        seen.add(key)
        by_model[model].append((vid, pid, profile))

        rotate = profile.get('rotate', None)
        if rotate is not None and (type(rotate) is not int or rotate not in (-1, 0, 1)):
            errors.append(f'{model or i}: rotate must be null, -1, 0, or 1')
        for field in ('width', 'height', 'fps'):
            value = profile.get(field)
            if not isinstance(value, (int, float)) or value <= 0:
                errors.append(f'{model or i}: {field} must be > 0')

required_models = ('JMC1S', 'JMC1M', 'JMC1L')
required_endpoints = {('0BDA', '300A'), ('0BDA', '300B')}
for model in required_models:
    entries = by_model.get(model, [])
    endpoints = {(vid, pid) for vid, pid, _ in entries}
    missing = required_endpoints - endpoints
    extra = endpoints - required_endpoints
    if missing:
        errors.append(f'{model}: missing A/B profiles: ' + ', '.join(f'{v}:{p}' for v, p in sorted(missing)))
    if extra:
        errors.append(f'{model}: unexpected VID/PID profiles: ' + ', '.join(f'{v}:{p}' for v, p in sorted(extra)))
    if len(entries) != 2:
        errors.append(f'{model}: expected exactly 2 profiles, got {len(entries)}')

if errors:
    print('\n'.join(errors))
    sys.exit(1)
print('camera model config check: PASS (strict model + VID + PID, independent A/B rotate profiles)')
