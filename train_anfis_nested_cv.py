•	KODE TRAINING ANFIS
import torch, torch.nn as nn, torch.optim as optim
import numpy as np, pandas as pd
import warnings, random, time
warnings.filterwarnings('ignore')
DATA_FILE = 'data_closed_loop.csv'
SEED, VAL_SEED = 42, 123
ARCH_GRID = [(mf,n) for mf in ['gaussian','triangular','trapezoidal'] for n in [3,5,7]]
FOU_GRID = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
LAMBDA_GRID = [0.01,0.03,0.05]
LR, LR_FT = 0.005, 0.001
EPOCHS_FINAL_T1, EPOCHS_FINAL_IT2 = 300, 300
def set_seed(s):
    random.seed(s); np.random.seed(s); torch.manual_seed(s); torch.cuda.manual_seed_all(s)
set_seed(SEED)
adaptive_bs = lambda n: int(np.clip(n//8, 8, 64))
adaptive_kfold = lambda n: (3,80,30) if n>=500 else (5,150,50)
def load_data(fp):
    df = pd.read_excel(fp) if str(fp).lower().endswith(('.xlsx','.xls')) else pd.read_csv(fp)
    rn = {'e_ph':'e_pH','de_ph':'de_pH','delay_ph':'delay_pH_ms','e_ec':'e_EC','de_ec':'de_EC',
          'delay_ec':'delay_EC_ms','e_do':'e_DO','de_do':'de_DO','suhu':'Suhu'}
    df = df.rename(columns={k:v for k,v in rn.items() if k in df.columns and v not in df.columns})
    if 'timestamp' in df.columns: df = df[df['timestamp']!='NTP_belum_sync'].copy()
    cols = ['e_suhu','de_suhu','pwm_kipas','e_pH','de_pH','delay_pH_ms','e_EC','de_EC','delay_EC_ms','e_DO','de_DO','pwm_aerator']
    for c in cols: df[c] = pd.to_numeric(df[c], errors='coerce')
    return df
def prepare_dataset(df, e_c, de_c, out_c, out_range, filt=None):
    d = filt(df) if filt else df
    d = d[d[out_c].notna() & d[e_c].notna() & d[de_c].notna()].copy()
    if len(d) < 30: return None, None, len(d)
    X = torch.stack([torch.tensor(d[e_c].values,dtype=torch.float32), torch.tensor(d[de_c].values,dtype=torch.float32)],dim=1)
    y = torch.tensor(d[out_c].values, dtype=torch.float32)
    y_norm = torch.clamp((y-out_range[0])/(out_range[1]-out_range[0]+1e-10), 0.0, 1.0)
    return X, y_norm, len(d)
SUBSYSTEM_CONFIGS = [
    {'name':'Suhu','e':'e_suhu','de':'de_suhu','out':'pwm_kipas','e_range':(-10.,10.),'de_range':(-3.,3.),'out_range':(0.,255.),'filter':None},
    {'name':'pH_Down','e':'e_pH','de':'de_pH','out':'delay_pH_ms','e_range':(-2.,2.),'de_range':(-.5,.5),'out_range':(0.,22000.),'filter':lambda d:d[d['e_pH']<-0.1]},
    {'name':'pH_Up','e':'e_pH','de':'de_pH','out':'delay_pH_ms','e_range':(-2.,2.),'de_range':(-.5,.5),'out_range':(0.,22000.),'filter':lambda d:d[d['e_pH']>0.1]},
    {'name':'EC','e':'e_EC','de':'de_EC','out':'delay_EC_ms','e_range':(-2.,2.),'de_range':(-.5,.5),'out_range':(0.,40000.),'filter':lambda d:d[d['e_EC']>0.1]},
    {'name':'DO','e':'e_DO','de':'de_DO','out':'pwm_aerator','e_range':(-3.,2.),'de_range':(-.5,.5),'out_range':(0.,255.),'filter':None},
]
def gaussian_mf(x,c,s): return torch.exp(-0.5*((x-c)/(s.abs()+1e-6))**2)
def triangular_mf(x,c,s):
    s = s.abs()+1e-6
    return torch.clamp(1.0-torch.abs(x-c)/s, 0.0, 1.0)
def trapezoidal_mf(x,c,s):
    s = s.abs()+1e-6; pl = 0.3*s; den = s-pl+1e-9
    rise, fall = (x-(c-s))/den, ((c+s)-x)/den
    return torch.clamp(torch.min(torch.min(rise,fall), torch.ones_like(x)), 0.0, 1.0)
MF_FUNCTIONS = {'gaussian':gaussian_mf,'triangular':triangular_mf,'trapezoidal':trapezoidal_mf}
class ANFIS_T1(nn.Module):
    def __init__(self, n_mf, e_r, de_r, out_r, mf_type='gaussian'):
        super().__init__()
        self.n_mf, self.n_rules, self.mf_type = n_mf, n_mf*n_mf, mf_type
        self.mf_func, self.e_range, self.de_range = MF_FUNCTIONS[mf_type], e_r, de_r
        self.c1 = nn.Parameter(torch.linspace(*e_r, n_mf)); self.c2 = nn.Parameter(torch.linspace(*de_r, n_mf))
        self.s1 = nn.Parameter(torch.full((n_mf,), (e_r[1]-e_r[0])/(2*n_mf)))
        self.s2 = nn.Parameter(torch.full((n_mf,), (de_r[1]-de_r[0])/(2*n_mf)))
        self.consequents = nn.Parameter(torch.zeros(self.n_rules))
    def _c(self): return torch.clamp(self.c1,*self.e_range), torch.clamp(self.c2,*self.de_range)
    def forward(self, x1, x2):
        c1,c2 = self._c()
        mu1 = self.mf_func(x1.unsqueeze(1), c1.unsqueeze(0), self.s1.unsqueeze(0))
        mu2 = self.mf_func(x2.unsqueeze(1), c2.unsqueeze(0), self.s2.unsqueeze(0))
        w = torch.bmm(mu1.unsqueeze(2), mu2.unsqueeze(1)).view(x1.shape[0], -1)
        return ((w/(w.sum(1,keepdim=True)+1e-10)) * self.consequents).sum(1)
    def get_params_dict(self):
        c1,c2 = self._c()
        return {'c1':c1.detach().numpy(),'c2':c2.detach().numpy(),'s1':self.s1.detach().abs().numpy(),          's2':self.s2.detach().abs().numpy(),'p':self.consequents.detach().numpy(),'mf_type':self.mf_type}
class IT2_ANFIS(nn.Module):
    def __init__(self, n_mf, e_r, de_r, out_r, mf_type, fou, t1p):
        super().__init__()
        self.n_mf, self.n_rules, self.mf_type = n_mf, n_mf*n_mf, mf_type
        self.mf_func, self.e_range, self.de_range, self.fou_ratio = MF_FUNCTIONS[mf_type], e_r, de_r, fou
        s1b, s2b = torch.tensor(t1p['s1']), torch.tensor(t1p['s2'])
        self.c1 = nn.Parameter(torch.tensor(t1p['c1']).clone()); self.c2 = nn.Parameter(torch.tensor(t1p['c2']).clone())
        self.s1_upper = nn.Parameter(s1b*(1+fou)); self.s2_upper = nn.Parameter(s2b*(1+fou))
        self.s1_lower = nn.Parameter(s1b*max(1-fou,0.05)); self.s2_lower = nn.Parameter(s2b*max(1-fou,0.05))
        self.consequents = nn.Parameter(torch.tensor(t1p['p']).clone())
    def _c(self): return torch.clamp(self.c1,*self.e_range), torch.clamp(self.c2,*self.de_range)
    def _fire(self, x1,x2,c1,c2,s1,s2):
        mu1 = self.mf_func(x1.unsqueeze(1), c1.unsqueeze(0), s1.unsqueeze(0))
        mu2 = self.mf_func(x2.unsqueeze(1), c2.unsqueeze(0), s2.unsqueeze(0))
        w = torch.bmm(mu1.unsqueeze(2), mu2.unsqueeze(1)).view(x1.shape[0], -1)
        return ((w/(w.sum(1,keepdim=True)+1e-10)) * self.consequents).sum(1)
    def forward(self, x1, x2):
        c1,c2 = self._c()
        return (self._fire(x1,x2,c1,c2,self.s1_upper,self.s2_upper) +
                self._fire(x1,x2,c1,c2,self.s1_lower,self.s2_lower)) / 2.0
    def get_params_dict(self):
        c1,c2 = self._c()
        return {'c1':c1.detach().numpy(),'c2':c2.detach().numpy(),                's1_upper':self.s1_upper.detach().abs().numpy(),'s1_lower':self.s1_lower.detach().abs().numpy(),        's2_upper':self.s2_upper.detach().abs().numpy(),'s2_lower':self.s2_lower.detach().abs().numpy(),
                'p':self.consequents.detach().numpy(),'mf_type':self.mf_type}
def t1_loss(pred, target, model, **_):
    reg = torch.mean(torch.relu(0.05-model.s1.abs())**2 + torch.relu(0.05-model.s2.abs())**2)
    mse = nn.functional.mse_loss(pred, target)
    return mse + 0.001*reg, mse
def it2_loss(pred, target, model, lambda_reg=0.01, lambda_shrink=0.002):
    mse = nn.functional.mse_loss(pred, target)
    reg_fou = torch.mean(torch.relu(model.s1_lower.abs()-model.s1_upper.abs())**2 +
                          torch.relu(model.s2_lower.abs()-model.s2_upper.abs())**2)
    reg_floor = torch.mean(torch.relu(0.05-model.s1_upper.abs())**2 + torch.relu(0.05-model.s1_lower.abs())**2)
    reg_shrink = torch.mean((model.s1_upper.abs()-model.s1_lower.abs())**2 + (model.s2_upper.abs()-model.s2_lower.abs())**2)
    return mse + lambda_reg*(reg_fou+reg_floor) + lambda_shrink*reg_shrink, mse
def train(model, X, y, epochs, lr, bs, loss_fn, patience=80, min_delta=1e-6, **loss_kwargs):
    opt = optim.Adam(model.parameters(), lr=lr)
    sch = optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
    n = X.shape[0]; best, no_imp, best_state = float('inf'), 0, None
    for _ in range(epochs):
        idx = torch.randperm(n); ep_loss, nb = 0.0, 0
        for i in range(0, n, bs):
            xb, yb = X[idx][i:i+bs], y[idx][i:i+bs]
            opt.zero_grad()
            loss, loss_mse = loss_fn(model(xb[:,0],xb[:,1]), yb, model, **loss_kwargs)
            loss.backward(); torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0); opt.step()
            ep_loss += loss_mse.item(); nb += 1
        sch.step()
        avg = ep_loss/nb
        if avg < best-min_delta: best, no_imp, best_state = avg, 0, {k:v.clone() for k,v in model.state_dict().items()}
        else:
            no_imp += 1
            if no_imp >= patience: break
    if best_state: model.load_state_dict(best_state)
    model.eval()
def eval_rmse(model, X, y):
    model.eval()
    with torch.no_grad(): return torch.sqrt(torch.mean((model(X[:,0],X[:,1])-y)**2)).item()
def eval_metrics(model, X, y_norm, out_range):
    model.eval(); o0,o1 = out_range
    with torch.no_grad(): pn = model(X[:,0], X[:,1])
    pred, y = pn*(o1-o0)+o0, y_norm*(o1-o0)+o0
    err = pred-y
    mask = torch.abs(y) >= 0.01*(o1-o0)
    mape = (torch.mean(torch.abs(err[mask])/torch.abs(y[mask])).item()*100) if mask.sum()>0 else float('nan')
    return {'rmse':torch.sqrt(torch.mean(err**2)).item(),'mae':torch.mean(torch.abs(err)).item(),'mape':mape,
            'mape_n_valid':int(mask.sum().item()),'mape_n_total':y.shape[0],
            'rmse_norm':torch.sqrt(torch.mean((pn-y_norm)**2)).item()}
def kfold_indices(n,k,seed):
    g = torch.Generator().manual_seed(seed)
    return torch.chunk(torch.randperm(n,generator=g), k)

def run_t1_arch_cv(X_dev, y_dev, cfg, bs, k, ep, pat):
    folds = kfold_indices(X_dev.shape[0], k, VAL_SEED)
    scores = {a: [] for a in ARCH_GRID}
    for i in range(k):
        va = folds[i]; tr = torch.cat([folds[j] for j in range(k) if j!=i])
        for mf, n_mf in ARCH_GRID:
            set_seed(SEED)
            m = ANFIS_T1(n_mf, cfg['e_range'], cfg['de_range'], cfg['out_range'], mf)
            train(m, X_dev[tr], y_dev[tr], ep, LR, min(bs,len(tr)), t1_loss, patience=pat)
            scores[(mf,n_mf)].append(eval_rmse(m, X_dev[va], y_dev[va]))
    mean = {a: float(np.mean(v)) for a,v in scores.items()}
    return min(mean, key=mean.get), mean
def run_subsystem(df, cfg):
    name = cfg['name']
    X, y, n = prepare_dataset(df, cfg['e'], cfg['de'], cfg['out'], cfg['out_range'], cfg['filter'])
    if X is None: print(f'SKIP {name}: data kurang ({n})'); return None
    bs = adaptive_bs(n)
    g = torch.Generator().manual_seed(SEED); idx = torch.randperm(n, generator=g)
    n_test = max(1, int(n*0.2))
    te, dv = idx[:n_test], idx[n_test:]
    X_dev, y_dev, X_te, y_te = X[dv], y[dv], X[te], y[te]
    k, ep_cv, pat_cv = adaptive_kfold(X_dev.shape[0])
    print(f'=== {name} (n={n}, dev={X_dev.shape[0]}, test={n_test}) ===')

    (mf_type, n_mf), _ = run_t1_arch_cv(X_dev, y_dev, cfg, bs, k, ep_cv, pat_cv)
    print(f'  Arsitektur T1: {mf_type}, N_MF={n_mf}')
    folds = kfold_indices(X_dev.shape[0], k, VAL_SEED)
    combo = {(f,l): [] for f in FOU_GRID for l in LAMBDA_GRID}
    for i in range(k):
        va = folds[i]; tr = torch.cat([folds[j] for j in range(k) if j!=i])
        set_seed(SEED)
        m1 = ANFIS_T1(n_mf, cfg['e_range'], cfg['de_range'], cfg['out_range'], mf_type)
        train(m1, X_dev[tr], y_dev[tr], ep_cv, LR, min(bs,len(tr)), t1_loss, patience=pat_cv)
        p1 = m1.get_params_dict()
        for fou in FOU_GRID:
            for lam in LAMBDA_GRID:
                set_seed(SEED)
                m2 = IT2_ANFIS(n_mf, cfg['e_range'], cfg['de_range'], cfg['out_range'], mf_type, fou, p1)
                train(m2, X_dev[tr], y_dev[tr], ep_cv, LR_FT, min(bs,len(tr)), it2_loss, patience=pat_cv, lambda_reg=lam)
                combo[(fou,lam)].append(eval_rmse(m2, X_dev[va], y_dev[va]))
    combo_mean = {c: float(np.mean(v)) for c,v in combo.items()}
    best_fou, best_lam = min(combo_mean, key=combo_mean.get)
    print(f'  (FOU, lambda) terbaik = ({best_fou}, {best_lam})')

    set_seed(SEED)
    m_t1 = ANFIS_T1(n_mf, cfg['e_range'], cfg['de_range'], cfg['out_range'], mf_type)
    train(m_t1, X_dev, y_dev, EPOCHS_FINAL_T1, LR, bs, t1_loss, patience=120)
    met_t1 = eval_metrics(m_t1, X_te, y_te, cfg['out_range'])
    p1f = m_t1.get_params_dict()
    set_seed(SEED)
    m_it2 = IT2_ANFIS(n_mf, cfg['e_range'], cfg['de_range'], cfg['out_range'], mf_type, best_fou, p1f)
    train(m_it2, X_dev, y_dev, EPOCHS_FINAL_IT2, LR_FT, bs, it2_loss, patience=120, lambda_reg=best_lam)
    met_it2 = eval_metrics(m_it2, X_te, y_te, cfg['out_range'])
    rng = cfg['out_range'][1]-cfg['out_range'][0]
    print(f"  RMSE_test%: T1={met_t1['rmse']/rng*100:.2f}  IT2={met_it2['rmse']/rng*100:.2f}")
    return {'name':name,'cfg':cfg,'best_fou':best_fou,'lambda_reg':best_lam,'best_mf_type':mf_type,'best_n_mf':n_mf,
            'model_t1':m_t1,'model_it2':m_it2,'met_t1_test':met_t1,'met_it2_test':met_it2}
df_data = load_data(DATA_FILE)
t0 = time.time(); results = {}
for cfg in SUBSYSTEM_CONFIGS:
    r = run_subsystem(df_data, cfg)
    if r: results[cfg['name']] = r
print(f'Selesai | {time.time()-t0:.1f} detik')
arr = lambda v: '{' + ', '.join(f'{x:.6f}f' for x in v) + '}'
def export_cpp(results, prefix, model_key, extra_sigma_keys, fname):
    lines = [f'// {prefix}-ANFIS Parameter -- Nested CV', '']
    for name, res in results.items():
        p = res[model_key].get_params_dict()
        o0, o1 = res['cfg']['out_range']; n = res['best_n_mf']; nr = n*n
        lines.append(f'#define {prefix}_{name}_N_MF {n}')
        lines.append(f'const float {prefix}_{name}_c1[{n}] = {arr(p["c1"])};')
        lines.append(f'const float {prefix}_{name}_c2[{n}] = {arr(p["c2"])};')
        for suffix, key in extra_sigma_keys:
            lines.append(f'const float {prefix}_{name}_{suffix}[{n}] = {arr(p[key])};')
        lines.append(f'const float {prefix}_{name}_p[{nr}] = {arr(p["p"])};')
        lines.append(f'const float {prefix}_{name}_out_min = {o0:.1f}f;')
        lines.append(f'const float {prefix}_{name}_out_max = {o1:.1f}f;\n')
    open(fname,'w').write('\n'.join(lines))
export_cpp(results, 'T1', 'model_t1', [('s1','s1'),('s2','s2')], 'ANFIS_T1_params_FIKSSS.h')
export_cpp(results, 'IT2', 'model_it2', [('s1u','s1_upper'),('s1l','s1_lower'),('s2u','s2_upper'),('s2l','s2_lower')], 'IT2_ANFIS_params_FIKSSS.h')

}
