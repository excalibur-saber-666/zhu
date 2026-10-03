"""E1/E4/E5 quantitative figures, retaining all frozen seeds and the original center."""
import argparse
import csv
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','D:/codex_agent/临时/2026-10-03_E1_E4_E5_revision/matplotlib')
from analyze_remaining_revision import OUTPUT
from plot_revision_experiments import plt, COLORS, LABELS, LINESTYLES, MARKERS, WIDTH_INCHES, panel
import numpy as np
from matplotlib.patches import Rectangle
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Arial','DejaVu Sans'],
    'font.size':8,'axes.labelsize':8,'axes.titlesize':8,'legend.fontsize':7,
    'svg.fonttype':'none'})


def read(output,stage,name):
    with (output/'tables'/stage/name).open(encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))


def save(fig,output,name):
    destination=output/'figures'; destination.mkdir(parents=True,exist_ok=True)
    fig.canvas.draw(); fig.canvas.draw(); fig.set_layout_engine('none')
    fig.savefig(destination/f'{name}.svg',facecolor='white')
    fig.savefig(destination/f'{name}.png',dpi=600,facecolor='white')
    plt.close(fig)


def e1(output):
    position=read(output,'E1','position_summary.csv'); health=read(output,'E1','health_summary.csv')
    event=read(output,'E1','event_summary.csv')
    fig,axes=plt.subplots(2,2,figsize=(WIDTH_INCHES,125/25.4),layout='constrained')
    definitions=[('a','Fault scenario: macro RMSE',position,'baseline_3f','RMSE3D',1,'m'),
        ('b','Healthy maneuver: alarm occupancy',health,'healthy_maneuver','AlarmHealthyOccupancy',100,'%'),
        ('c','Fault scenario: conditional detection delay',event,'baseline_3f','DetectionDelay',1,'s'),
        ('d','Healthy maneuver: isolation occupancy',health,'healthy_maneuver','IsolationHealthyOccupancy',100,'%')]
    for ax,(label,title,rows,scenario,metric,multiplier,unit) in zip(axes.flat,definitions):
        selected=[r for r in rows if r['Scenario']==scenario and r['Method']=='cusum_fgo']
        if rows is position: selected=[r for r in selected if r['Aggregation']=='macro']
        if rows is health: selected=[r for r in selected if r['EdgeScope']=='leader' and r['Phase']=='all']
        values=np.zeros((3,3)); sd=np.zeros((3,3))
        for i,h in enumerate((3,5,7)):
            for j,k in enumerate((.25,.5,.75)):
                point=f'h{h}_k{k:g}'.replace('.','p'); row=next(r for r in selected if r['Point']==point)
                assert int(row[metric+'_N'])==50
                values[i,j]=float(row[metric+'_Mean'])*multiplier; sd[i,j]=float(row[metric+'_SD'])*multiplier
        im=ax.imshow(values,origin='lower',cmap='cividis',aspect='auto')
        mid=(values.min()+values.max())/2
        for i in range(3):
            for j in range(3):
                ax.text(j,i,f'{values[i,j]:.2f}\n± {sd[i,j]:.2f}',ha='center',va='center',
                        fontsize=8,color='white' if values[i,j]<mid else 'black')
        ax.add_patch(Rectangle((.5,.5),1,1,fill=False,edgecolor='white',lw=2))
        ax.set_xticks(range(3),['0.25','0.50','0.75']); ax.set_yticks(range(3),['3','5','7'])
        ax.set_xlabel('Drift parameter $\\kappa$'); ax.set_ylabel('Alarm threshold $h$')
        panel(ax,label,title); fig.colorbar(im,ax=ax,label=unit,fraction=.05,pad=.03)
    fig.suptitle('CUSUM-FGO sensitivity; mean ± SD of 50 seeds per point; white box: original parameters',fontsize=8)
    save(fig,output,'E1_parameter_sensitivity')


def e4(output):
    rows=[r for r in read(output,'E4','runtime_summary.csv') if r['Phase']=='steady']
    fig,axes=plt.subplots(2,2,figsize=(WIDTH_INCHES,125/25.4),layout='constrained')
    definitions=[('a','Method processing (log scale)','AlgorithmSeconds_Mean',1000,'Mean algorithm runtime (ms)'),
        ('b','Complete online cycle (log scale)','EndToEndSeconds_Mean',1000,'Mean end-to-end runtime (ms)'),
        ('c','Online tail latency (log scale)','EndToEndSeconds_P99',1000,'Per-seed P99 end-to-end (ms)'),
        ('d','Graph convergence','GNMean',1,'Mean Gauss–Newton iterations')]
    scenes=('size_2f','baseline_3f','size_5f'); x=[2,3,5]
    for ax,(label,title,metric,multiplier,ylabel) in zip(axes.flat,definitions):
        methods=('fgo','cusum_fgo') if metric=='GNMean' else tuple(COLORS)
        for method in methods:
            chosen=[next(r for r in rows if r['Scenario']==s and r['Method']==method) for s in scenes]
            assert all(int(r[metric+'_N'])==10 for r in chosen)
            values=[float(r[metric+'_Mean'])*multiplier for r in chosen]
            deviations=[float(r[metric+'_SD'])*multiplier for r in chosen]
            assert np.all(np.asarray(values)>0)
            # Log-axis means and mean +/- SD bounds must stay strictly positive.
            if metric!='GNMean': assert np.all(np.asarray(values)-np.asarray(deviations)>0)
            ax.errorbar(x,values,yerr=deviations,color=COLORS[method],marker=MARKERS[method],
                linestyle=LINESTYLES[method],ms=3,capsize=2,label=LABELS[method])
        ax.set_xticks(x); ax.set_xlabel('Number of followers (3 leaders)'); ax.set_ylabel(ylabel)
        if metric=='GNMean': ax.set_ylim(bottom=0)
        else: ax.set_yscale('log')
        panel(ax,label,title)
    axes[0,0].legend(ncol=2,loc='best',handlelength=1.5,columnspacing=.7)
    axes[1,1].legend(loc='best',handlelength=1.5)
    fig.suptitle('One MATLAB process/thread; steady phase 10–600 s; mean ± SD across 10 seeds',fontsize=8)
    save(fig,output,'E4_runtime_convergence')


def e5(output):
    rows=read(output,'E5','metrics_per_seed.csv')
    fig,axes=plt.subplots(1,2,figsize=(WIDTH_INCHES,75/25.4),layout='constrained')
    for ax,metric,label in zip(axes,('RMSE3D','CDF95'),('a','b')):
        for x,point in enumerate(('full','no_isolation','no_soft_weighting')):
            selected=sorted((r for r in rows if r['Point']==point and r['Aggregation']=='macro'),key=lambda r:int(r['Seed']))
            assert len(selected)==50
            values=np.array([float(r[metric]) for r in selected]); color=list(COLORS.values())[3-x]
            ax.scatter(x+np.linspace(-.13,.13,50),values,s=9,alpha=.55,color=color,edgecolors='none')
            ax.errorbar(x,values.mean(),yerr=values.std(ddof=1),color='black',marker='_',capsize=4)
        ax.set_xticks([0,1,2],['Full','No confirmed\nisolation','No soft\nweighting'])
        ax.set_ylabel(('Macro 3D RMSE' if metric=='RMSE3D' else 'Macro follower CDF95')+' (m)')
        panel(ax,label,'All 50 paired seeds retained')
    fig.suptitle('3F3L baseline NLOS; original parameters; black line: mean ± SD',fontsize=8)
    save(fig,output,'E5_mechanism_ablation')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--stage',choices=['E1','E4','E5'],required=True)
    p.add_argument('--output',type=Path,default=OUTPUT); args=p.parse_args()
    globals()[args.stage.lower()](args.output); print('Exported',args.stage)
