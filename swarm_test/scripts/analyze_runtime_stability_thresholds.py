#!/usr/bin/env python3
"""Sensitivity analysis for tracking settling and persistent SECBF slack."""
import argparse, bisect, csv, math
from collections import defaultdict
from pathlib import Path

def rows(path):
    with path.open(newline='', encoding='utf-8') as f: return list(csv.DictReader(f))

def f(row, key, default=float('nan')):
    try: return float(row.get(key, default))
    except (TypeError, ValueError): return default

def sustained_start(samples, start_t, predicate, hold):
    begin=None; last=None
    for t,v in samples:
        if t < start_t: continue
        if predicate(v):
            if begin is None or last is None or t-last > 0.5: begin=t
            if t-begin >= hold: return begin
            last=t
        else: begin=last=None
    return None

def longest_episode(samples, predicate):
    best=0.; begin=last=None
    for t,v in samples:
        if predicate(v):
            if begin is None or last is None or t-last > 0.5: begin=t
            best=max(best,t-begin); last=t
        else: begin=last=None
    return best

def analyze_trial(run):
    planner=rows(run/'planner_log.csv'); obstacle=rows(run/'obstacle_log.csv')
    track=[(f(r,'t'),f(r,'tracking_error')) for r in planner]
    track=[x for x in track if all(math.isfinite(v) for v in x)]
    slack=[(f(r,'t'),f(r,'slack_max',f(r,'slack',0.))) for r in planner]
    slack=[x for x in slack if all(math.isfinite(v) for v in x)]
    grouped=defaultdict(list)
    for r in obstacle:
        if math.hypot(f(r,'vx',0),f(r,'vy',0))>1e-3: grouped[r.get('id','')].append((f(r,'t'),f(r,'d_i')))
    closest=[min(v,key=lambda x:x[1])[0] for v in grouped.values() if v]
    completion=max(closest)+1.0 if closest else (track[0][0] if track else float('nan'))
    out={'interaction_completion_t':completion,'tracking_rmse_m':math.sqrt(sum(v*v for _,v in track)/len(track)) if track else float('nan'),'tracking_error_max_m':max((v for _,v in track),default=float('nan'))}
    for threshold in (0.20,0.30,0.40):
        for hold in (0.5,1.0):
            start=sustained_start(track,completion,lambda v,t=threshold:v<=t,hold)
            out[f'settle_{threshold:.2f}_{hold:.1f}_s']=start-completion if start is not None else float('nan')
            out[f'settle_{threshold:.2f}_{hold:.1f}_within5']=int(start is not None and start-completion<=5.0)
    for threshold in (1e-3,3e-3,5e-3):
        longest=longest_episode(slack,lambda v,t=threshold:v>t)
        out[f'slack_run_gt_{threshold:g}_s']=longest
        for hold in (0.5,1.0,2.0): out[f'persistent_{threshold:g}_{hold:.1f}']=int(longest>=hold)
    return out

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--input-root',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); a=ap.parse_args()
    result=[]
    for run in sorted(p for p in a.input_root.iterdir() if p.is_dir() and not p.name.startswith('.')):
        if not (run/'summary.csv').exists(): continue
        summary=next(csv.DictReader((run/'summary.csv').open()))
        row={'scenario':summary['scenario'],'baseline':summary['baseline'],'run':run.name,**analyze_trial(run)}; result.append(row)
    if not result: raise SystemExit('no completed trials')
    with a.output.open('w',newline='',encoding='utf-8') as fobj:
        w=csv.DictWriter(fobj,fieldnames=list(result[0])); w.writeheader(); w.writerows(result)
    print(f'Wrote {a.output} ({len(result)} trials)')
if __name__=='__main__': main()
