import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import allantools 

# Load data
ddata = pd.read_csv('/home/kaviak/catkin_ws/src/gps_driver/src/5hrdata.csv')
gx = ddata.iloc[:, 0].values
gy = ddata.iloc[:, 1].values
gz = ddata.iloc[:, 2].values

data=gz
nb=200
a='Z'



Fs = 40
t0 = 1/Fs
theta = np.cumsum(data) * t0
L = len(theta)
maxM = 2**np.floor(np.log2(L/2))
maxNumM = nb
m = np.ceil(np.logspace(np.log10(1), np.log10(maxM), maxNumM)).astype(int)
m = np.unique(m)  # Remove duplicates
tau = m * t0
avar = np.zeros(len(m))
for i, mi in enumerate(m):
    differ = (theta[2*mi:L] - 2*theta[mi:L-mi] + theta[:L-2*mi])
    nanIndices = np.isnan(differ)
    differ[nanIndices] = 0.00
    avar[i] = np.sum(differ**2)
avar = avar / (2 * tau**2 * (L - 2*m))
adev = np.sqrt(avar)



# Plot Allan Deviation
plt.figure()
plt.loglog(tau, adev)
plt.title(a+'-axis Gyro Allan Deviation')
plt.xlabel(r'$\tau$')
plt.ylabel(r'$\sigma(\tau)$')
plt.legend(['Manual Calculation'])
plt.grid(True, which="both", ls="-", color='0.65')
plt.xscale('log')
plt.yscale('log')
plt.savefig(a+'-axis_Gyro_Allan_Deviation.png', dpi=300)

# Angle Random Walk
slope = -0.5
logtau = np.log10(tau)
logadev = np.log10(adev)
dlogadev = np.diff(logadev) / np.diff(logtau)
i = np.argmin(np.abs(dlogadev - slope))
b = logadev[i] - slope*logtau[i]
logN = slope*np.log10(1) + b
N = 10**logN

# Plot Angle Random Walk
tauN = 1
lineN = N / np.sqrt(tau)
plt.figure()
plt.loglog(tau, adev, tau, lineN, '--', tauN, N, 'o')
plt.title(a+'-axis Gyro Allan Deviation: Angle Random Walk(N)')
plt.xlabel(r'$\tau$')
plt.ylabel(r'$\sigma(\tau)$')
plt.legend([r'$\sigma$', r'$\sigma_N$'])
plt.text(tauN, N, 'N(t=1)')
plt.grid(True, which="both", ls="-", color='0.65')

plt.xscale('log')
plt.yscale('log')
plt.savefig(a+'-axis_Gyro_Allan_Deviation_Angle_Random_Walk.png', dpi=300)

# Rate Random Walk
slope = 0.5
dlogadev = np.diff(logadev) / np.diff(logtau)
i = np.argmin(np.abs(dlogadev - slope))
b = logadev[i] - slope*logtau[i]
logK = slope*np.log10(3) + b
K = 10**logK
# Plot Rate Random Walk
tauK = 3
lineK = (10**logK)* np.sqrt(tau/3)

idx = np.argwhere(np.diff(np.sign(adev - lineK))).flatten()                     
nonzero_idxs = np.where(adev[idx] != 0)[0]

min_nonzero_idx = idx[nonzero_idxs[np.argmin(adev[idx[nonzero_idxs]])]]
tauk=tau[min_nonzero_idx]
k=adev[min_nonzero_idx]
plt.figure()
plt.loglog(tau, adev, tau, lineK, '--',tauk ,k , 'o')

plt.title(a+'-axis Gyro Allan Deviation: Rate Random Walk(K)')
plt.xlabel(r'$\tau$')
plt.ylabel(r'$\sigma(\tau)$')
plt.legend([r'$\sigma$', r'$\sigma_K$'])
plt.text(tauk, k, 'K')
plt.grid(True, which="both", ls="-", color='0.65')

plt.xscale('log')
plt.yscale('log')
plt.savefig(a+'-axis_Gyro_Allan_Deviation_Rate_Random_Walk.png', dpi=300)


# Plot Rate Random Walk

# Bias Instability
slope = 0
dlogadev = np.diff(logadev) / np.diff(logtau)
i = np.argmin(np.abs(dlogadev - slope))
b = logadev[i] - slope*logtau[i]
scfB = np.sqrt(2*np.log(2)/np.pi)
logB = b - np.log10(scfB)
B = 10**logB

# Plot Bias Instability
tauB = tau[i]
lineB = B * scfB * np.ones_like(tau)
plt.figure()
plt.loglog(tau, adev, tau, lineB, '--', tauB, scfB*B, 'o')
plt.title(a+'-axis Gyro Allan Deviation: Bias instability(B)')
plt.xlabel(r'$\tau$')
plt.ylabel(r'$\sigma(\tau)$')
plt.legend([r'$\sigma$', r'$\sigma_B$'])
plt.text(tauB, scfB*B, '0.664B')
plt.grid(True, which="both", ls="-", color='0.65')

plt.xscale('log')
plt.yscale('log')
plt.savefig(a+'-axis_Gyro_Allan_Deviation_Bias_instability.png', dpi=300)

# Noise Parameters

tauParams = [tauN, tauk, tauB]
params = [N, k, scfB*B]


plt.figure()
plt.loglog(tau, adev, label='$\sigma (rad/s)$')
plt.loglog(tau, lineN, '--')
plt.loglog(tau, lineK, '--')
plt.loglog(tau, lineB, '--')
plt.scatter(tauParams, params, marker='o')

for i, txt in enumerate(['N(t=1)','K', '0.664B']):
    plt.text(tauParams[i], params[i], txt)
plt.title(a+'-axis Gyro Allan Deviation: All Noise Parameters')
plt.xlabel(r'$\tau$')
plt.ylabel(r'$\sigma(\tau)$')
plt.legend([r'$\sigma (rad/s)$', r'$\sigma_N ((rad/s)/\sqrt{Hz})$', 
            r'$\sigma_K ((rad/s)\sqrt{Hz})$', r'$\sigma_B (rad/s)$'])
plt.grid(True, which="both", ls="-", color='0.65')

plt.xscale('log')
plt.yscale('log')
plt.savefig(a+'-axis_Gyro_Allan_Deviation_All_Noise_Parameters.png', dpi=300)
adata=[]

plt.show()



