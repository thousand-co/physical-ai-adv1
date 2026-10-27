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
    if report.get('mode')=='conveyor_sorting':
        parcels=report.get('parcels',[])
        if len(parcels)!=report.get('expected_count') or not parcels:
            raise ValueError('Incomplete sorting sequence')
        for parcel in parcels:
            expected='delivery_tray' if parcel['expected_color']=='blue' else 'reject_tray'
            if (parcel['predicted_color']!=parcel['expected_color'] or parcel['destination']!=expected
                    or not all(parcel.get(k) for k in ['physical_placement_confirmed','lift_confirmed','transport_confirmed','floor_contact_confirmed','retained_at_end'])):
                raise ValueError('Sorting parcel failed verification: '+str(parcel))
    print('Verified physical delivery:',path)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('evidence');p.add_argument('--newer-than',type=float,default=0);a=p.parse_args()
    try:check(a.evidence,a.newer_than)
    except (ValueError,OSError) as exc:print(str(exc),file=sys.stderr);sys.exit(1)
