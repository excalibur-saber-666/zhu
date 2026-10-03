"""Reproducible E3/E2 figures; all Monte Carlo rows are retained.

Figure contract: quantitative grids. State-chain panels establish what the
online detector and admission policy actually did; aggregate panels show
seed-level dispersion and limitations. Fixed seed 1 is illustrative only.
SVG retains editable text; PNG is a 600 dpi companion. Default width is
180 mm, with 8 pt text. No time sample is treated as an independent trial.
"""
import argparse
import csv
import json
import os
from pathlib import Path

from analyze_revision_experiments import DEFAULT_OUTPUT, write_csv
import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "D:/codex_agent/临时/2026-10-03_E3_E2_revision/matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({"font.family":"sans-serif","font.sans-serif":["Arial","DejaVu Sans"],
    "font.size":8,"axes.labelsize":8,"axes.titlesize":8,"legend.fontsize":7,
    "svg.fonttype":"none","axes.spines.top":False,
    "axes.spines.right":False,"axes.linewidth":.7,"lines.linewidth":1.2,"legend.frameon":False})
COLORS={"ekf":"#8C8C8C","fgo":"#597FA0","cusum_ekf":"#C6975D","cusum_fgo":"#427C71"}
LABELS={"ekf":"EKF","fgo":"FGO","cusum_ekf":"CUSUM-EKF","cusum_fgo":"CUSUM-FGO"}
LINESTYLES={"ekf":":","fgo":"--","cusum_ekf":"-.","cusum_fgo":"-"}
MARKERS={"ekf":"s","fgo":"^","cusum_ekf":"D","cusum_fgo":"o"}
WIDTH_INCHES=7.086614173228346  # Exactly 180 mm; within double-column layouts.


def table(output,stage,name):
    with (output/"tables"/stage/name).open(encoding="utf-8-sig",newline="") as f:
        return list(csv.DictReader(f))


def save(fig,output,name):
    destination=output/"figures"
    destination.mkdir(parents=True,exist_ok=True)
    # Freeze one Agg layout before switching export renderers. Repeated
    # constrained-layout passes across SVG/PNG can otherwise shift labels.
    fig.canvas.draw()
    fig.canvas.draw()
    fig.set_layout_engine("none")
    fig.savefig(destination/f"{name}.svg",facecolor="white")
    fig.savefig(destination/f"{name}.png",dpi=600,facecolor="white")
    plt.close(fig)


def trace(output,scenario,seed,method="cusum_fgo"):
    return dict(np.load(output/"derived"/scenario/f"seed_{seed:04d}_{method}_trace.npz"))


def panel(ax,label,title=""):
    ax.set_title(f"{label}  {title}",loc="left",fontweight="bold",pad=6)
    ax.tick_params(width=.6,length=3)


def state_figure(output,events,seed,event_id,name,failure=False):
    event=next(e for e in events if int(e["Seed"])==seed and int(e["Event"])==event_id and e["Method"]=="cusum_fgo")
    tr=trace(output,"baseline_3f",seed)
    follower=int(event["Follower"]); leader=int(event["Leader"])
    F=int(tr["followers"]); control=2 if leader!=2 else 1
    start=float(event["FaultStart"]); end=float(event["FaultEnd"])
    left=max(1,start-12); right=min(600,end+(65 if failure else 42))
    fig,axes=plt.subplots(4,2,figsize=(WIDTH_INCHES,170/25.4),sharex=True,layout="constrained")
    rows=[]
    for c,L in enumerate((leader,control)):
        j=np.flatnonzero(np.all(tr["pairs"]==[follower,F+L],axis=1))[0]
        t=tr["time"]; mask=(t>=left)&(t<=right)
        for a in axes[:,c]:
            a.axvspan(start,end,color="#D4A177",alpha=.18,zorder=0)
            a.axvline(end+30,color=".5",ls=":",lw=.8)
            a.set_xlim(left,right)
        axes[0,c].plot(t[mask],tr["z"][mask,j],color=COLORS["cusum_fgo"])
        axes[0,c].axhline(0,color=".7",lw=.5)
        axes[0,c].set_ylabel("Normalized innovation")
        axes[1,c].plot(t[mask],tr["cplus"][mask,j],label="$C^+$",color=COLORS["cusum_fgo"])
        axes[1,c].plot(t[mask],tr["cminus"][mask,j],label="$C^-$",color=COLORS["cusum_ekf"],ls="--")
        axes[1,c].axhline(5,color=".35",ls=":",lw=.8,label="$h=5$")
        axes[1,c].set_ylabel("CUSUM evidence")
        axes[1,c].legend(loc="upper right",ncol=3,handlelength=1.5,columnspacing=.8)
        axes[2,c].plot(t[mask],tr["weight"][mask,j],color=COLORS["cusum_fgo"],label="Weight")
        axes[2,c].step(t[mask],tr["admitted"][mask,j].astype(int),where="post",color=".4",ls="--",label="Admission")
        axes[2,c].step(t[mask],tr["alarm"][mask,j].astype(int),where="post",color=COLORS["cusum_ekf"],ls=":",label="Alarm")
        axes[2,c].set_ylim(-.07,1.12); axes[2,c].set_ylabel("Weight / state")
        axes[2,c].legend(loc="center right",ncol=1,handlelength=1.5)
        axes[3,c].step(t[mask],tr["retained_factors"][mask,j],where="post",color=".5",label="All retained")
        axes[3,c].step(t[mask],tr["retained_fault_factors"][mask,j],where="post",color=COLORS["cusum_ekf"],label="Fault factors")
        axes[3,c].set_ylabel("Factors in window"); axes[3,c].set_xlabel("Time (s)")
        axes[3,c].set_ylim(-.5,10.7); axes[3,c].legend(loc="lower right",handlelength=1.5)
        edge_title=f"F{follower}-L{L}: "+("fault edge" if c==0 else "healthy control")
        for r in range(4):
            panel(axes[r,c],chr(ord('a')+2*r+c),edge_title if r==0 else "")
        for k in np.flatnonzero(mask):
            rows.append(dict(Seed=seed,Event=event_id,Follower=follower,Leader=L,Time=float(t[k]),
                **{key:float(tr[key][k,j]) for key in ("z","cplus","cminus","weight","alarm","admitted",
                    "true_range","prediction","frozen","rate_before","rate_after","fault_bias",
                    "retained_factors","retained_fault_factors")}))
    qualifier="first failure in seed/event order" if failure else "pre-specified illustration"
    fig.suptitle(f"CUSUM-FGO, seed {seed}, event {event_id} ({qualifier})",fontsize=9)
    write_csv(output/"tables"/"figure_sources"/f"{name}.csv",rows)
    save(fig,output,name)


def e3_figures(output):
    events=table(output,"E3","events_per_seed.csv")
    seeds=table(output,"E3","event_seed_summary.csv")
    health=table(output,"E3","health_per_seed.csv")
    state_figure(output,events,1,1,"E3_state_chain")
    failed=sorted((e for e in events if e["Method"]=="cusum_fgo" and e["RecoveryStatus"]=="failed"),
                  key=lambda e:(int(e["Seed"]),int(e["Event"])))
    if failed:
        state_figure(output,events,int(failed[0]["Seed"]),int(failed[0]["Event"]),"E3_recovery_failure",True)
    fig,axes=plt.subplots(1,3,figsize=(WIDTH_INCHES,65/25.4),layout="constrained")
    base=sorted((r for r in seeds if r["Scenario"]=="baseline_3f" and r["Method"]=="cusum_fgo"),key=lambda r:int(r['Seed']))
    for x,(field,label,color) in enumerate((("DetectionDelay","Detection",COLORS['fgo']),
            ("IsolationDelay","Isolation",COLORS['cusum_ekf']),("RecoveryDelay","Recovery",COLORS['cusum_fgo']))):
        values=np.array([float(r[field]) for r in base if r[field]])
        jitter=np.linspace(-.13,.13,len(values))
        axes[0].scatter(x+jitter,values,s=9,alpha=.55,color=color,edgecolors="none")
        axes[0].errorbar(x,values.mean(),yerr=values.std(ddof=1),color="black",marker="_",capsize=3,lw=1)
    axes[0].set_xticks(range(3),["Detection","Isolation","Recovery"],rotation=25)
    axes[0].set_ylabel("Mean delay within seed (s)")
    panel(axes[0],"a","Conditional delays; n = 50 seeds")
    failure=np.array([float(r['RecoveryFailureRate'])*100 for r in base])
    axes[1].scatter(np.linspace(-.13,.13,len(failure)),failure,s=10,color=COLORS['cusum_fgo'],alpha=.55)
    axes[1].errorbar(0,failure.mean(),yerr=failure.std(ddof=1),color="black",marker="_",capsize=4)
    axes[1].set_xticks([0],["Within 30 s"]); axes[1].set_ylabel("Recovery failure / seed (%)")
    panel(axes[1],"b","Failures retained; n = 50 seeds")
    for x,scenario in enumerate(("baseline_3f","healthy_maneuver")):
        group=sorted((r for r in health if r['Scenario']==scenario and r['Method']=='cusum_fgo' and
                       r['EdgeScope']=='leader' and r['Phase']=='all'),key=lambda r:int(r['Seed']))
        values=np.array([float(r['IsolationHealthyOccupancy'])*100 for r in group])
        axes[2].scatter(x+np.linspace(-.13,.13,len(values)),values,s=9,alpha=.55,color=COLORS['cusum_fgo'])
        axes[2].errorbar(x,values.mean(),yerr=values.std(ddof=1),color="black",marker="_",capsize=3)
    axes[2].set_xticks([0,1],["Baseline\nhealthy epochs","Healthy\nmaneuver"])
    axes[2].set_ylabel("Healthy isolation occupancy (%)")
    panel(axes[2],"c","Includes post-fault persistence")
    save(fig,output,"E3_seed_statistics")
    # Healthy dynamic pressure: all 50 seeds and all 9 leader edges are used.
    first=trace(output,"healthy_maneuver",1)
    leader=first['pairs'][:,1]>int(first['followers']); t=first['time']
    maximum=[]; occupancy=[]; source_rows=[]
    for seed in range(1,51):
        tr=trace(output,"healthy_maneuver",seed)
        maximum.append(np.max(np.maximum(tr['cplus'][:,leader],tr['cminus'][:,leader]),axis=1))
        occupancy.append(np.mean(tr['isolated'][:,leader],axis=1)*100)
    maximum=np.asarray(maximum); occupancy=np.asarray(occupancy)
    fig,axes=plt.subplots(3,1,figsize=(WIDTH_INCHES,130/25.4),sharex=True,layout="constrained")
    excursion=first['true_range'][:,leader]-first['true_range'][0,leader]
    axes[0].plot(t,excursion,color=COLORS['fgo'],alpha=.55,lw=.9)
    axes[0].set_ylabel("Range change (m)"); panel(axes[0],"a","Nine leader-follower edges; seed 1")
    axes[1].plot(t,maximum.T,color=COLORS['cusum_fgo'],alpha=.15,lw=.55)
    axes[1].plot(t,np.median(maximum,axis=0),color="black",label="Median across seeds")
    axes[1].axhline(5,color=COLORS['cusum_ekf'],ls="--",label="Alarm threshold")
    axes[1].set_ylabel("Maximum edge CUSUM")
    axes[1].legend(loc="upper left",ncol=2); panel(axes[1],"b","All 50 seeds; maximum over nine leader edges")
    axes[2].plot(t,occupancy.mean(axis=0),color=COLORS['cusum_fgo'])
    axes[2].set_ylabel("Isolated healthy edges (%)"); axes[2].set_xlabel("Time (s)")
    panel(axes[2],"c","Across 50 seeds × 9 edges at each epoch")
    for a in axes:
        a.axvspan(260,460,color=COLORS['fgo'],alpha=.07)
        for start,end in [(300,315),(400,415)]:
            a.axvspan(start,end,color=COLORS['cusum_ekf'],alpha=.16)
        a.set_xlim(0,600)
    for k,time in enumerate(t):
        for s in range(50):
            source_rows.append(dict(Seed=s+1,Time=float(time),MaxLeaderCUSUM=float(maximum[s,k]),
                                   LeaderIsolationPercent=float(occupancy[s,k])))
    write_csv(output/"tables"/"figure_sources"/"E3_healthy_maneuver.csv",source_rows)
    write_csv(output/"tables"/"figure_sources"/"E3_healthy_range_excursion.csv",
        [dict(Time=float(t[k]),Edge=f"F{p[0]}-L{p[1]-3}",RangeChange=float(excursion[k,j]))
         for j,p in enumerate(first['pairs'][leader]) for k in range(len(t))])
    save(fig,output,"E3_healthy_maneuver")


def e2_figures(output):
    rows=table(output,"E2","position_summary.csv")
    for experiment,scenarios,xvalues,xlabel in (
        ("E2a_formation_size",["size_2f","baseline_3f","size_5f"],[2,3,5],"Number of followers (3 leaders)"),
        ("E2b_NLOS_intensity",["nlos_weak","baseline_3f","nlos_strong"],[.5,1,1.5],"NLOS amplitude multiplier")):
        fig,axes=plt.subplots(2,2,figsize=(WIDTH_INCHES,125/25.4),layout="constrained")
        for r,aggregation in enumerate(("pooled","macro")):
            for c,metric in enumerate(("RMSE3D","CDF95")):
                ax=axes[r,c]
                for method in COLORS:
                    selected=[next(a for a in rows if a['Scenario']==s and a['Method']==method and
                               a['Aggregation']==aggregation) for s in scenarios]
                    values=[float(a[metric+'_Mean']) for a in selected]
                    deviations=[float(a[metric+'_SD']) for a in selected]
                    ax.errorbar(xvalues,values,yerr=deviations,marker=MARKERS[method],ms=3,capsize=2,
                                linestyle=LINESTYLES[method],color=COLORS[method],label=LABELS[method])
                ax.set_xticks(xvalues); ax.set_xlabel(xlabel)
                ax.set_ylabel(("3D RMSE" if metric=='RMSE3D' else "95th percentile")+" (m)")
                title=("Pooled overall" if aggregation=='pooled' else "Follower equal macro-average")
                panel(ax,chr(ord('a')+r*2+c),title)
                ax.set_ylim(bottom=0)
        axes[0,0].legend(loc="best",ncol=2,columnspacing=.7,handlelength=1.5)
        fig.suptitle("Mean ± SD across 50 paired seeds",fontsize=9)
        save(fig,output,experiment)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument('--stage',choices=['E3','E2'],required=True)
    args=parser.parse_args()
    if args.stage=='E3': e3_figures(args.output)
    else: e2_figures(args.output)
    print(f"Figures exported for {args.stage}")


if __name__=='__main__': main()
