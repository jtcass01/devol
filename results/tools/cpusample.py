"""Samples cumulative CPU time (utime+stime) of every process every 2 s; writes the last value per pid."""
import os, sys, time, json
out = sys.argv[1]; tck = os.sysconf('SC_CLK_TCK'); seen = {}; t0 = time.time()
while True:
    for p in os.listdir('/proc'):
        if not p.isdigit(): continue
        try:
            st = open(f'/proc/{p}/stat').read(); cmd = open(f'/proc/{p}/cmdline').read().replace('\0', ' ').strip()
        except OSError: continue
        f = st[st.rfind(')') + 2:].split()
        cpu = (int(f[11]) + int(f[12])) / tck
        e = seen.setdefault(p, {'cmd': cmd[:300], 'first': time.time() - t0})
        e['cpu'] = cpu; e['last'] = time.time() - t0
    json.dump(seen, open(out, 'w'))
    if os.path.exists(out + '.stop'): break
    time.sleep(2)
