"""Extended audits: delay-augmented and noise-augmented re-simulations + gain
sensitivity + LC-plant mismatch bound.
Generates results/ext_delay.csv, ext_noise.csv, ext_gain.csv,
ext_LC_mismatch.csv and their figures.
"""
import csv, json, itertools, os
import numpy as np
from sim_core import BuckBoost, DigitalPID, all_metrics
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT="results"
os.makedirs(OUT+"/figures", exist_ok=True)
gains=json.load(open(f"{OUT}/params.json"))["pid_gains"]
FSW=100e3
VREF=-12.0
T_NOM=0.06

plt.rcParams.update({"font.size":9, "axes.grid":True, "grid.alpha":0.3, "figure.dpi":140})

def run_one(ts_mult, comp_delay_s=0.0, adc_noise_sigma=0.0, adc_bits=16, pwm_bits=16, vin=12.0, c=220e-6):
    pid=DigitalPID(*gains, 1/FSW)
    bb=BuckBoost(vin=vin, c=c, ts_mult=ts_mult, adc_bits=adc_bits, pwm_bits=pwm_bits, pid=pid,
                 comp_delay_s=comp_delay_s, adc_noise_sigma=adc_noise_sigma)
    # seed for noise runs to make reproducible
    if adc_noise_sigma>0:
        np.random.seed(0)
    r=bb.run(T_NOM, VREF)
    m=all_metrics(r, VREF)
    return r,m

# 1) Delay sweep: Ts 1,1.5,2,2.5,3,3.5,4 with delays 0,11.4us,65us
delays=[0.0, 11.4e-6, 65e-6]
ts_list=[1,1.5,2,2.5,3,3.5,4]
rows=[]
for d in delays:
    for ts in ts_list:
        _,m=run_one(ts, comp_delay_s=d)
        rows.append(dict(delay_us=d*1e6, ts=ts, t_settle_ms=m["t_settle"]*1e3, ss_error_mV=m["ss_error"]*1e3,
                         ripple_mV=m["ripple_pp"]*1e3, iae=m["iae"], dcm=m["dcm_cycles"],
                         settled=int(m["t_settle"]*1e3<50)))
        print(f"delay {d*1e6:.1f}us Ts {ts}x settle {m['t_settle']*1e3:.2f} ss {m['ss_error']*1e3:.1f} ripple {m['ripple_pp']*1e3:.1f} dcm {m['dcm_cycles']}")

# save
with open(f"{OUT}/ext_delay.csv","w",newline="") as f:
    w=csv.DictWriter(f, fieldnames=rows[0].keys())
    w.writeheader(); w.writerows(rows)

# figure delay vs Ts
fig,axs=plt.subplots(1,2, figsize=(7.5,3.2))
for d,c in zip(delays, ["tab:blue","tab:orange","tab:green"]):
    sub=[r for r in rows if r["delay_us"]==d*1e6]
    xs=[r["ts"] for r in sub]
    axs[0].plot(xs, [r["t_settle_ms"] for r in sub], "o-", color=c, label=f"{d*1e6:.4g}us" if d>0 else "0 (ideal)")
    axs[1].semilogy(xs, [abs(r["ss_error_mV"]) for r in sub], "o-", color=c, label=f"{d*1e6:.4g}us" if d>0 else "0")
axs[0].set(xlabel="Ts/Tsw", ylabel="2% settling [ms]"); axs[0].legend(fontsize=7); axs[0].set_ylim(0,62)
axs[1].set(xlabel="Ts/Tsw", ylabel="|ss error| [mV]")
fig.suptitle("Delay-augmented boundary: zero-delay vs STM32 bare (11.4us) vs ESP32 Arduino (65us)")
fig.tight_layout(); fig.savefig(f"{OUT}/figures/fig_delay_boundary.png"); plt.close(fig)
print("fig_delay_boundary.png saved")

# 2) Noise sweep: fix Ts=1x, add Gaussian sigma 0,0.1,0.5,1.0,1.5 V (output-frame),
#    repeated for ADC resolutions 16/12/10/8 bit so the "quantization spread
#    collapses under noise" claim is directly visible in the data.
sigmas=[0.0,0.1,0.5,1.0,1.5]
noise_bits=[16,12,10,8]
rows2=[]
for sigma in sigmas:
    for bits in noise_bits:
        _,m=run_one(1, adc_noise_sigma=sigma, adc_bits=bits)
        rows2.append(dict(sigma=sigma, adc=bits, t_settle_ms=m["t_settle"]*1e3, ss_error_mV=m["ss_error"]*1e3,
                          ripple_mV=m["ripple_pp"]*1e3, iae=m["iae"], dcm=m["dcm_cycles"]))
        print(f"noise sigma {sigma} V ADC {bits}-bit: settle {m['t_settle']*1e3:.2f} ripple {m['ripple_pp']*1e3:.1f}")

with open(f"{OUT}/ext_noise.csv","w",newline="") as f:
    w=csv.DictWriter(f, fieldnames=rows2[0].keys()); w.writeheader(); w.writerows(rows2)

fig,ax=plt.subplots(figsize=(5,3))
colors={16:"tab:blue",12:"tab:orange",10:"tab:green",8:"tab:red"}
for bits in noise_bits:
    sub=[r for r in rows2 if r["adc"]==bits]
    ax.plot([r["sigma"] for r in sub], [r["ripple_mV"] for r in sub], "o-",
            color=colors[bits], label=f"ADC {bits}-bit")
ax.legend(fontsize=7)
ax.set(xlabel="Added ADC noise sigma [V, output-frame]", ylabel="peak-to-peak ripple [mV]")
ax.set_title("Noise masks quantization: 12/10/8-bit spread collapses")
fig.tight_layout(); fig.savefig(f"{OUT}/figures/fig_noise_masking.png"); plt.close(fig)
print("fig_noise_masking.png saved")

# 3) Gain sensitivity: Kp beyond edge, Kd beyond edge
kp_ext=[0.02,0.05,0.08,0.10]
kd_ext=[0.0,1e-5,2e-5,5e-5]
rows3=[]
# fix Ki=100
for kp in kp_ext:
    for kd in kd_ext:
        # reuse DigitalPID but override kp/kd, keep Ki=100
        # find boundary: scan Ts 1..4
        t_bound=None
        for ts in ts_list:
            pid=DigitalPID(kp,100,kd,1/FSW)
            bb=BuckBoost(ts_mult=ts, pid=pid)
            r=bb.run(T_NOM,VREF)
            m=all_metrics(r,VREF)
            if m["t_settle"]*1e3>=50 or abs(m["ss_error"])>1.0:
                t_bound=ts
                break
        # fallback: if none hits, bound >4
        rows3.append(dict(kp=kp, kd=kd, boundary_Ts=t_bound if t_bound else 4.5, kp_kd=f"{kp}/{kd}"))
        print(f"Kp {kp} Kd {kd} boundary {t_bound}")

with open(f"{OUT}/ext_gain.csv","w",newline="") as f:
    w=csv.DictWriter(f, fieldnames=rows3[0].keys()); w.writeheader(); w.writerows(rows3)

# heatmap of boundary vs Kp,Kd
import numpy as np
mat=np.zeros((len(kp_ext), len(kd_ext)))
for r in rows3:
    i=kp_ext.index(r["kp"]); j=kd_ext.index(r["kd"])
    mat[i,j]=r["boundary_Ts"]
fig,ax=plt.subplots(figsize=(5,3.5))
im=ax.imshow(mat, cmap="viridis", vmin=2.5, vmax=4.5, origin="lower")
ax.set_xticks(range(len(kd_ext))); ax.set_xticklabels([str(v) for v in kd_ext])
ax.set_yticks(range(len(kp_ext))); ax.set_yticklabels([str(v) for v in kp_ext])
ax.set_xlabel("Kd"); ax.set_ylabel("Kp")
for i in range(len(kp_ext)):
    for j in range(len(kd_ext)):
        ax.text(j,i, f"{mat[i,j]:.1f}", ha="center", va="center", color="white", fontsize=7)
fig.colorbar(im, ax=ax, label="first unusable Ts/Tsw")
ax.set_title("Gain sensitivity: boundary Ts/Tsw (first never-settled)")
fig.tight_layout(); fig.savefig(f"{OUT}/figures/fig_gain_sense.png"); plt.close(fig)
print("fig_gain_sense.png saved")

# 4) LC-plant mismatch bound: nominal 220 uF vs deferred-hardware 2.2 uF
#    (f0 1.07 kHz vs 10.7 kHz) with the frozen gains, Ts/Tsw 1..5.
C_LIST=[(220.0, 220e-6), (2.2, 2.2e-6)]
ts_lc=[1,1.5,2,2.5,3,3.5,4,5]
rows4=[]
for c_uf, c_farad in C_LIST:
    for ts in ts_lc:
        _,m=run_one(ts, c=c_farad)
        rows4.append(dict(C_uF=c_uf, ts=ts, t_ms=m["t_settle"]*1e3, ss=m["ss_error"]*1e3,
                          ripple=m["ripple_pp"]*1e3, dcm=m["dcm_cycles"],
                          settled=int(m["t_settle"]*1e3<50)))
        print(f"LC C={c_uf}uF Ts {ts}x settle {m['t_settle']*1e3:.2f} ss {m['ss_error']*1e3:.1f} ripple {m['ripple_pp']*1e3:.1f} dcm {m['dcm_cycles']}")

with open(f"{OUT}/ext_LC_mismatch.csv","w",newline="") as f:
    w=csv.DictWriter(f, fieldnames=rows4[0].keys()); w.writeheader(); w.writerows(rows4)

fig,ax=plt.subplots(figsize=(5.6,3))
for (c_uf,_),col in zip(C_LIST, ["tab:blue","tab:red"]):
    sub=[r for r in rows4 if r["C_uF"]==c_uf]
    c_farad = 220e-6 if c_uf==220.0 else 2.2e-6
    f0=1/(2*np.pi*np.sqrt(100e-6*c_farad))/1e3
    ax.plot([r["ts"] for r in sub], [r["t_ms"] for r in sub], "o-", color=col,
            label=f"C={c_uf:.1f}\u03bcF  f0={f0:.2f}kHz")
ax.axhline(50, color="k", ls=":", lw=0.8)
ax.set(xlabel="Ts/Tsw", ylabel="2% settling [ms]")
ax.set_title("LC mismatch (frozen gains): 220\u03bcF sim vs 2.2\u03bcF HW", fontsize=10)
ax.legend(fontsize=7, loc="center right")
fig.tight_layout(); fig.savefig(f"{OUT}/figures/fig_LC_mismatch.png"); plt.close(fig)
print("fig_LC_mismatch.png saved")
print("ALL EXTENDED AUDITS DONE")
