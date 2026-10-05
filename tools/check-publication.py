#!/usr/bin/env python3
"""Small public-source privacy regression check; not a substitute for manual review.
Use `git ls-files` after checkout so dependencies, runtime storage and private ignored
files are not read. Release reviews must separately scan all committed Git blobs,
PDF streams/metadata and the pixels of images.
"""
from pathlib import Path
import re, subprocess, sys, gzip, hashlib
ROOT = Path(__file__).resolve().parents[1]
try:
    names = subprocess.check_output(['git', '-C', str(ROOT), 'ls-files', '-z']).decode().split('\0')
    if not any(names): raise subprocess.CalledProcessError(1, 'git ls-files')
except subprocess.CalledProcessError:
    names = [str(p.relative_to(ROOT)) for p in ROOT.rglob('*') if p.is_file() and 'node_modules' not in p.parts and '.git' not in p.parts]
RULES = {
    'credential': rb'(?:sk-ant-[A-Za-z0-9_-]{20,}|gh[pousr]_[A-Za-z0-9]{30,}|AKIA[A-Z0-9]{16})',
    'private-key': rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
    'personal-user-path': rb'/Users/[A-Za-z0-9_.-]+/|[A-Z]:\\Users\\[A-Za-z0-9_.-]+\\',
    'private-tailnet': rb'https?://[A-Za-z0-9.-]+\.ts\.net',
}
# Exact upstream-verified bundles contain inlined WASM/base64 that can randomly
# resemble token prefixes. Only this credential rule is excepted at these hashes;
# private-key, user-path and tailnet checks still apply.
VERIFIED_BUNDLES = {'app/vendor/opencv/opencv.js.gz': 'd531c9478836420bf60574c78da813b88f400d143c1ed070d14a1a03ef65b5e7', 'app/vendor/spark/spark.module.js': 'ce6c34c33137fbf1b98753482326ab93ddeba6e945656b4cb15fd82f8d85211b'}
PRIVATE_SUFFIXES = {'.h264', '.mov', '.mp4', '.webm', '.ply', '.splat', '.spz', '.pcap', '.pcapng', '.pem', '.key', '.jsonl'}
findings=[]
for name in names:
    if not name: continue
    p=ROOT/name
    if p.is_symlink(): findings.append((name,'symlink'));continue
    if p.suffix in PRIVATE_SUFFIXES or p.name == '.env' or (p.name.startswith('.env.') and p.name != '.env.example'):
        findings.append((name,'private-file-type'))
    if not p.is_file(): continue
    b=p.read_bytes()
    verified_bundle = VERIFIED_BUNDLES.get(name) == hashlib.sha256(b).hexdigest()
    if p.suffix == '.gz': b=gzip.decompress(b)
    for rule,pattern in RULES.items():
        if rule == 'credential' and verified_bundle: continue
        if re.search(pattern,b): findings.append((name,rule))
for name,rule in findings: print(f'{name}: {rule}')
print(f'Privacy regression: {len([n for n in names if n])} public files; {len(findings)} findings')
sys.exit(bool(findings))
