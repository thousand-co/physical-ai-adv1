"""Reject missing, stale, failed or physically unverified delivery evidence."""
import argparse
import json
from pathlib import Path
import sys

def check(path,newer_than=0):
    path=Path(path)
    if not path.exists() or path.stat().st_mtime<newer_than:
        raise ValueError('No fresh delivery evidence: '+str(path))
    report=json.loads(path.read_text())
    if report.get('final_state')!='done' or not report.get('success'):
        raise ValueError('Physical delivery failed: '+str(report.get('events',[])[-1:]))
    if not all(report.get(key) for key in ['lift_confirmed','transport_confirmed','floor_contact_confirmed']):
        raise ValueError('Physical delivery checkpoints were not confirmed')
    if report.get('disable_grip'):
        raise ValueError('Open-gripper control must not be reported as a delivery success')
    print('Verified physical delivery:',path)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('evidence');p.add_argument('--newer-than',type=float,default=0);a=p.parse_args()
    try:check(a.evidence,a.newer_than)
    except (ValueError,OSError) as exc:print(str(exc),file=sys.stderr);sys.exit(1)
