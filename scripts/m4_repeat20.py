#!/usr/bin/env python3
"""Twenty fresh-world repetitions of the unchanged SEM M4 trial (simulation only).

Run only with no other Gazebo worlds active. Never operates a real robot.
No threshold tuning, retries, or source changes; all attempted outcomes are kept.
"""
from __future__ import annotations

import csv
import hashlib
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SDF = Path.home() / 'stop_evidence_results/sem_m4.sdf'
OUT = Path.home() / 'stop_evidence_results/m4_repeat20'
N = 20
EXPECTED = {'normal': 'STOP_EVIDENCED', 'misrouted': 'STOP_FAILED',
            'missing_lidar': 'UNVERIFIABLE'}
RUNNER_CODE = (
    'import sys; from pathlib import Path; '
    'sys.path.insert(0, sys.argv[2]); import m4_live_trial; '
    'm4_live_trial.OUT = Path(sys.argv[1]); '
    'sys.exit(m4_live_trial.run())'
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def worlds():
    proc = subprocess.run(['gz', 'service', '-l'], capture_output=True,
                          text=True, timeout=15, check=True)
    return sorted(set(re.findall(r'^/world/([^/]+)/state$',
                                 proc.stdout, flags=re.MULTILINE)))


def wait_world(server, timeout_s=90):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if server.poll() is not None:
            raise RuntimeError('Gazebo exited during startup; see run gazebo.log')
        active = worlds()
        if active == ['sem_m4']:
            time.sleep(2)  # allow the LiDAR publisher to initialize
            return
        if active:
            raise RuntimeError(f'Unexpected concurrent Gazebo worlds: {active}')
        time.sleep(1)
    raise RuntimeError('Gazebo world did not start before bounded timeout')


def stop_owned(server):
    if server.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(server.pid, sig)
        except ProcessLookupError:
            break
        try:
            server.wait(timeout=8)
            return
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError('Could not stop owned Gazebo server process group')


def wait_clear(timeout_s=25):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        active = worlds()
        if not active:
            return
        time.sleep(1)
    raise RuntimeError(f'Gazebo worlds remained after owned server exit: {worlds()}')


def write_report():
    rows = []
    for p in sorted(OUT.glob('run_??.meta.json')):
        record = json.loads(p.read_text())
        f = record.get('findings') or {}
        rows.append({
            'trial': record['trial'],
            'result': record.get('result', 'NO_RESULT'),
            'exit_code': record.get('exit_code'),
            'normal': f.get('normal', {}).get('status', ''),
            'misrouted': f.get('misrouted', {}).get('status', ''),
            'missing_lidar': f.get('missing_lidar', {}).get('status', ''),
            'misrouted_displacement_m': f.get('misrouted', {}).get('displacement_after_bad_stop_m', ''),
            'error': record.get('error', ''),
        })
    fields = ['trial', 'result', 'exit_code', 'normal', 'misrouted',
              'missing_lidar', 'misrouted_displacement_m', 'error']
    with (OUT / 'results.csv').open('w', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    status = {}
    for scenario, expected in EXPECTED.items():
        observed = [r[scenario] for r in rows if r[scenario]]
        counts = Counter(observed)
        status[scenario] = {
            'expected': expected,
            'observed_runs': len(observed),
            'expected_status_count': counts[expected],
            'other_status_count': len(observed) - counts[expected],
            'status_counts': dict(counts),
        }
    distances = [float(r['misrouted_displacement_m']) for r in rows
                 if r['misrouted_displacement_m'] != '']
    summary = {
        'campaign': 'M4 20 fresh-world repetitions (simulation only)',
        'planned_complete_trials': N,
        'attempted_trials': len(rows),
        'complete_three_scenario_trials': sum(
            r['result'] == 'M4_STRAIGHT_LINE_GATES_PASSED' and r['exit_code'] == 0
            for r in rows),
        'results_by_scenario': status,
        'misrouted_displacement_m': {
            'samples': len(distances),
            'mean': statistics.mean(distances) if distances else None,
            'sample_stddev': statistics.stdev(distances) if len(distances) > 1 else None,
            'min': min(distances) if distances else None,
            'max': max(distances) if distances else None,
        },
        'misrouted_false_stop_evidence_count': sum(
            r['misrouted'] == 'STOP_EVIDENCED' for r in rows),
        'note': 'One M4 execution contains all three scenarios. No detection-latency timing or hardware claims. Trial failures are included, not retried.',
    }
    (OUT / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def main():
    if not os.environ.get('ROS_DISTRO'):
        raise SystemExit('ABORT: source the ROS 2 environment first')
    if not SDF.is_file():
        raise SystemExit(f'ABORT: missing {SDF}; generate via scripts/m4_make_world.py')
    if worlds():
        raise SystemExit('ABORT: shut down existing Gazebo worlds before starting the 20-run campaign')

    fingerprint = {
        'ros_distro': os.environ['ROS_DISTRO'],
        'source_sha256': digest(ROOT / 'scripts/m4_live_trial.py'),
        'world_sha256': digest(SDF),
        'runner_sha256': digest(Path(__file__)),
        'run_count': N,
        'fresh_world_each_trial': True,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    config = OUT / 'campaign.json'
    if config.is_file() and json.loads(config.read_text()) != fingerprint:
        raise SystemExit('ABORT: source/world/runner fingerprint changed; existing campaign remains frozen')
    if not config.exists():
        config.write_text(json.dumps(fingerprint, indent=2) + '\n')
    print(f'Output: {OUT}\nEach attempt starts and stops its own Gazebo server.', flush=True)

    for i in range(1, N + 1):
        prefix = OUT / f'run_{i:02d}'
        meta = Path(str(prefix) + '.meta.json')
        result = Path(str(prefix) + '.json')
        simlog = Path(str(prefix) + '.gazebo.log')
        triallog = Path(str(prefix) + '.trial.log')
        if meta.exists():
            print(f'[{i}/{N}] Already recorded; keeping original attempt', flush=True)
            continue
        if any(p.exists() for p in (result, simlog, triallog)):
            raise SystemExit(f'ABORT: partial files without {meta}; inspect {prefix} before resuming')
        if worlds():
            raise SystemExit('ABORT: unexpected Gazebo world present before run')
        record = {'trial': i, 'result': 'NO_RESULT', 'exit_code': None}
        server = None
        print(f'[{i}/{N}] Starting fresh-world M4 trial', flush=True)
        try:
            with simlog.open('w') as gzout:
                server = subprocess.Popen(
                    ['gz', 'sim', '-s', '-r', str(SDF)], cwd=ROOT,
                    stdout=gzout, stderr=subprocess.STDOUT, start_new_session=True)
                wait_world(server)
                with triallog.open('w') as rout:
                    try:
                        proc = subprocess.run(
                            [sys.executable, '-c', RUNNER_CODE, str(result), str(ROOT / 'scripts')],
                            cwd=ROOT, stdout=rout, stderr=subprocess.STDOUT,
                            timeout=240, check=False)
                        record['exit_code'] = proc.returncode
                    except subprocess.TimeoutExpired:
                        record['error'] = 'M4 script exceeded 240 s (timeout); inspect and stop any orphaned ROS bridge before continuing'
                if result.is_file():
                    data = json.loads(result.read_text())
                    record.update({'result': data.get('result'),
                                   'findings': data.get('findings', {}),
                                   'first_failure': data.get('first_failure')})
        except Exception as ex:
            record['error'] = str(ex)
        finally:
            try:
                if server is not None:
                    stop_owned(server)
                wait_clear()
            except Exception as ex:
                record['error'] = (record.get('error', '') + '; cleanup: ' + str(ex)).strip('; ')
            meta.write_text(json.dumps(record, indent=2) + '\n')
            summary = write_report()
            print(f"[{i}/{N}] result={record['result']} exit={record['exit_code']} error={record.get('error', '-')}", flush=True)
            if 'error' in record and (record['result'] == 'NO_RESULT' or 'cleanup:' in record['error']):
                print('Infrastructure/cleanup issue: preserving evidence and stopping the campaign.', flush=True)
                return 2
    summary = write_report()
    print('\n=== M4 20-RUN SUMMARY ===')
    print(json.dumps(summary, indent=2))
    print(f"\nReports: {OUT / 'summary.json'} and {OUT / 'results.csv'}")
    return 0 if summary['complete_three_scenario_trials'] == N else 1


if __name__ == '__main__':
    sys.exit(main())
