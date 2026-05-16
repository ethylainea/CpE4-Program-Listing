import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestClassifier, RandomForestRegressor
from sklearn.model_selection import train_test_split, StratifiedKFold, KFold
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix, mean_absolute_error
from sklearn.preprocessing import LabelEncoder

# ---------------------------------------------------------------
# CLASSIFIER MAE HELPER
# Encodes string labels as ordered integers, then computes MAE.
# For binary classifiers (soil): 0=not_suitable, 1=suitable
#   MAE of 0.0 = perfect, 1.0 = every prediction wrong
# For ordinal classifiers (slope): 0=stable, 1=pre_failure, 2=failure_imminent
#   MAE of 1.0 means predictions are off by one risk level on average
# ---------------------------------------------------------------
def classifier_mae(y_true, y_pred, order):
    """order: list of class labels from lowest to highest rank"""
    rank = {label: i for i, label in enumerate(order)}
    true_enc = np.array([rank[v] for v in y_true], dtype=float)
    pred_enc = np.array([rank[v] for v in y_pred], dtype=float)
    return mean_absolute_error(true_enc, pred_enc)

np.random.seed(42)

# ---------------------------------------------------------------
# LOAD DATA
# ---------------------------------------------------------------
CSV_FILE = 'site.csv'

try:
    df = pd.read_csv(CSV_FILE, encoding='utf-8-sig')
    if 'Delta_Tilt' in df.columns and 'DeltaTilt_deg' not in df.columns:
        df = df.rename(columns={'Delta_Tilt': 'DeltaTilt_deg'})
    print(f"Loaded {CSV_FILE}: {len(df)} rows")
except FileNotFoundError:
    print(f"ERROR: {CSV_FILE} not found.")
    exit(1)

required_cols = ['Moisture_%', 'EC_uS_cm', 'pH', 'DeltaTilt_deg', 'Soil_Suitability', 'Slope_Risk', 'Slips']
for col in required_cols:
    if col not in df.columns:
        print(f"ERROR: Missing column '{col}'")
        exit(1)

raw_df = df[required_cols].copy()
raw_df['DeltaTilt_deg'] = raw_df['DeltaTilt_deg'].abs()

print(f"Slope_Risk:       {raw_df['Slope_Risk'].value_counts().to_dict()}")
print(f"Soil_Suitability: {raw_df['Soil_Suitability'].value_counts().to_dict()}")

# ---------------------------------------------------------------
# FEATURE ENGINEERING (clean — no noise injection)
# ---------------------------------------------------------------
raw_df['EC_dScm']        = raw_df['EC_uS_cm'] / 1000.0
raw_df['shear_strength'] = ((raw_df['Moisture_%'] - 50) / 30) * 20
raw_df['RAR']            = raw_df['shear_strength'] / (1.2 * 75000)
raw_df['density']        = (raw_df['RAR'] * 1000) / 0.005

# ---------------------------------------------------------------
# RF HYPERPARAMETERS — tuned for generalization, NOT memorization
#
# Key regularization knobs vs. original:
#   max_depth        : 3  (was 5)  — shallower trees generalize better
#   min_samples_split: 20 (was 10) — require more samples before splitting
#   min_samples_leaf : 10 (was 5)  — require more samples at each leaf
#   max_features     : 'sqrt'      — unchanged; already good
#   n_estimators     : 50 (was 10) — more trees = lower variance via averaging
#   max_samples      : 0.8         — each tree sees only 80% of data (subsampling)
#
# These together force trees to learn broader patterns rather than
# memorizing individual sensor readings.
# ---------------------------------------------------------------
RF_PARAMS = dict(
    n_estimators=50,
    max_depth=3,
    min_samples_split=20,
    min_samples_leaf=10,
    max_features='sqrt',
    bootstrap=True,
    max_samples=0.8,
    random_state=42,
    n_jobs=-1,
)

skf = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)

# ---------------------------------------------------------------
# SOIL SUITABILITY MODEL  (features: EC_dScm, pH)
# ---------------------------------------------------------------
print("\n--- Soil Suitability Model ---")

X_soil = raw_df[['EC_dScm', 'pH']].values
y_soil = raw_df['Soil_Suitability'].values

X_str, X_ste, y_str, y_ste = train_test_split(
    X_soil, y_soil, test_size=0.2, random_state=42, stratify=y_soil
)

soil_cv = []
for fold, (tr, va) in enumerate(skf.split(X_str, y_str), 1):
    m = RandomForestClassifier(**RF_PARAMS)
    m.fit(X_str[tr], y_str[tr])
    train_acc = m.score(X_str[tr], y_str[tr])
    val_acc   = m.score(X_str[va], y_str[va])
    soil_cv.append(val_acc)
    print(f"  Fold {fold}: train={train_acc*100:.2f}%  val={val_acc*100:.2f}%  gap={abs(train_acc-val_acc)*100:.2f}%")

soil_model = RandomForestClassifier(**RF_PARAMS)
soil_model.fit(X_str, y_str)
soil_pred = soil_model.predict(X_ste)

SOIL_ORDER = ['not_suitable', 'suitable']  # 0 → 1 (ordinal rank)

soil_mae = classifier_mae(y_ste, soil_pred, SOIL_ORDER)

print(f"CV mean : {np.mean(soil_cv)*100:.2f}% (±{np.std(soil_cv)*100:.2f}%)")
print(f"Test    : {accuracy_score(y_ste, soil_pred)*100:.2f}%")
print(f"Test MAE: {soil_mae:.4f}  (0.0=perfect, 1.0=all wrong — binary scale)")
print(classification_report(y_ste, soil_pred, digits=3))
print(confusion_matrix(y_ste, soil_pred))
print(f"Feature importances: EC_dScm={soil_model.feature_importances_[0]:.4f}  pH={soil_model.feature_importances_[1]:.4f}")

# ---------------------------------------------------------------
# SLOPE RISK MODEL  (features: DeltaTilt_deg, Moisture_%)
# ---------------------------------------------------------------
print("\n--- Slope Risk Model ---")

X_slope = raw_df[['DeltaTilt_deg', 'Moisture_%']].values
y_slope = raw_df['Slope_Risk'].values

X_sltr, X_slte, y_sltr, y_slte = train_test_split(
    X_slope, y_slope, test_size=0.2, random_state=42, stratify=y_slope
)

slope_params = dict(**RF_PARAMS, class_weight='balanced')
slope_cv = []
for fold, (tr, va) in enumerate(skf.split(X_sltr, y_sltr), 1):
    m = RandomForestClassifier(**slope_params)
    m.fit(X_sltr[tr], y_sltr[tr])
    train_acc = m.score(X_sltr[tr], y_sltr[tr])
    val_acc   = m.score(X_sltr[va], y_sltr[va])
    slope_cv.append(val_acc)
    print(f"  Fold {fold}: train={train_acc*100:.2f}%  val={val_acc*100:.2f}%  gap={abs(train_acc-val_acc)*100:.2f}%")

slope_model = RandomForestClassifier(**slope_params)
slope_model.fit(X_sltr, y_sltr)
slope_pred = slope_model.predict(X_slte)

SLOPE_ORDER = ['stable', 'pre_failure', 'failure_imminent']  # 0 → 1 → 2

slope_mae = classifier_mae(y_slte, slope_pred, SLOPE_ORDER)

print(f"CV mean : {np.mean(slope_cv)*100:.2f}% (±{np.std(slope_cv)*100:.2f}%)")
print(f"Test    : {accuracy_score(y_slte, slope_pred)*100:.2f}%")
print(f"Test MAE: {slope_mae:.4f}  (0.0=perfect, 1.0=off by one risk level, 2.0=all wrong)")
print(classification_report(y_slte, slope_pred, digits=3))
print(confusion_matrix(y_slte, slope_pred))
print(f"Feature importances: DeltaTilt={slope_model.feature_importances_[0]:.4f}  Moisture={slope_model.feature_importances_[1]:.4f}")

# ---------------------------------------------------------------
# SLIP COUNT REGRESSION MODEL  (features: Moisture_%, DeltaTilt_deg)
# trained on non-zero rows only
# ---------------------------------------------------------------
print("\n--- Slip Count Regression Model ---")

slip_mask = raw_df['Slips'] > 0
slip_df   = raw_df[slip_mask].copy()
y_slips   = raw_df.loc[slip_mask, 'Slips'].values
print(f"Training rows (non-zero slips): {len(slip_df)}")
print(f"Slips range: {y_slips.min()} - {y_slips.max()}  mean={y_slips.mean():.1f}")

X_slp = slip_df[['Moisture_%', 'DeltaTilt_deg']].values

X_dtr, X_dte, y_dtr, y_dte = train_test_split(
    X_slp, y_slips, test_size=0.2, random_state=42
)

kf = KFold(n_splits=5, shuffle=True, random_state=42)
slp_cv_r2 = []
for fold, (tr, va) in enumerate(kf.split(X_dtr), 1):
    m = RandomForestRegressor(**RF_PARAMS)
    m.fit(X_dtr[tr], y_dtr[tr])
    train_r2 = m.score(X_dtr[tr], y_dtr[tr])
    val_r2   = m.score(X_dtr[va], y_dtr[va])
    mae      = mean_absolute_error(y_dtr[va], m.predict(X_dtr[va]))
    rmse     = np.sqrt(np.mean((y_dtr[va] - m.predict(X_dtr[va])) ** 2))
    slp_cv_r2.append(val_r2)
    print(f"  Fold {fold}: train_R²={train_r2*100:.2f}%  val_R²={val_r2*100:.2f}%  gap={abs(train_r2-val_r2)*100:.2f}%  MAE={mae:.2f}  RMSE={rmse:.2f}")

slip_model = RandomForestRegressor(**RF_PARAMS)
slip_model.fit(X_dtr, y_dtr)

y_dte_pred = slip_model.predict(X_dte)
slp_mae    = mean_absolute_error(y_dte, y_dte_pred)
slp_rmse   = np.sqrt(np.mean((y_dte - y_dte_pred) ** 2))

print(f"CV R²    : {np.mean(slp_cv_r2)*100:.2f}% (±{np.std(slp_cv_r2)*100:.2f}%)")
print(f"Test R²  : {slip_model.score(X_dte, y_dte)*100:.2f}%")
print(f"Test MAE : {slp_mae:.2f} slips/m²")
print(f"Test RMSE: {slp_rmse:.2f} slips/m²  (gap={slp_rmse - slp_mae:.2f})")
print(f"Feature importances: Moisture={slip_model.feature_importances_[0]:.4f}  DeltaTilt={slip_model.feature_importances_[1]:.4f}")

# ---------------------------------------------------------------
# OVERFITTING HEALTH CHECK
# Prints a warning if train/val gap is suspiciously large (>15%)
# ---------------------------------------------------------------
print("\n--- Overfitting Health Check ---")
final_soil_train  = soil_model.score(X_str,  y_str)
final_slope_train = slope_model.score(X_sltr, y_sltr)
final_slip_train  = slip_model.score(X_dtr,  y_dtr)
final_soil_test   = accuracy_score(y_ste,  soil_pred)
final_slope_test  = accuracy_score(y_slte, slope_pred)
final_slip_test   = slip_model.score(X_dte, y_dte)

for label, train_s, test_s in [
    ("Soil ",  final_soil_train,  final_soil_test),
    ("Slope",  final_slope_train, final_slope_test),
    ("Slip ",  final_slip_train,  final_slip_test),
]:
    gap  = abs(train_s - test_s)
    flag = " WARNING: OVERFIT" if gap > 0.15 else " OK"
    print(f"  {label}: train={train_s*100:.1f}%  test={test_s*100:.1f}%  gap={gap*100:.1f}% [{flag}]")

# ---------------------------------------------------------------
# EXPORT -> combined_model.py  (pure Python, no dependencies)
# ---------------------------------------------------------------
def export_tree(tree, feature_names, class_labels=None, indent=8, is_regressor=False):
    t_ = tree.tree_
    lines = []

    def recurse(node, depth):
        pad = " " * (indent + depth * 4)
        if t_.feature[node] != -2:
            feat   = feature_names[t_.feature[node]]
            thresh = round(float(t_.threshold[node]), 6)
            lines.append(f"{pad}if features['{feat}'] <= {thresh}:")
            recurse(t_.children_left[node],  depth + 1)
            lines.append(f"{pad}else:")
            recurse(t_.children_right[node], depth + 1)
        else:
            if is_regressor:
                val = round(float(t_.value[node][0][0]), 4)
                lines.append(f"{pad}return {val}")
            else:
                label = class_labels[int(np.argmax(t_.value[node]))]
                lines.append(f"{pad}return '{label}'")

    recurse(0, 0)
    return "\n".join(lines)

def export_rf_classifier(rf, feature_names, name):
    class_labels = list(rf.classes_)
    lines = [
        f"# features: {feature_names}",
        f"# classes : {class_labels}",
        f"def {name}_predict(features):",
        f"    votes = {{}}",
    ]
    for i, tree in enumerate(rf.estimators_):
        fn = f"_{name}_tree_{i}"
        lines += [
            f"",
            f"    def {fn}():",
            export_tree(tree, feature_names, class_labels),
            f"",
            f"    v = {fn}()",
            f"    votes[v] = votes.get(v, 0) + 1",
        ]
    lines += [
        f"",
        f"    return max(votes, key=votes.get)",
        f"",
    ]
    return "\n".join(lines)

def export_rf_regressor(rf, feature_names, name):
    lines = [
        f"# features: {feature_names}",
        f"def {name}_predict(features):",
        f"    total = 0.0",
    ]
    for i, tree in enumerate(rf.estimators_):
        fn = f"_{name}_tree_{i}"
        lines += [
            f"",
            f"    def {fn}():",
            export_tree(tree, feature_names, is_regressor=True),
            f"",
            f"    total += {fn}()",
        ]
    lines += [
        f"",
        f"    return total / {len(rf.estimators_)}",
        f"",
    ]
    return "\n".join(lines)

soil_code  = export_rf_classifier(soil_model,  ['EC_dScm', 'pH'],            'soil_model')
slope_code = export_rf_classifier(slope_model, ['delta_tilt', 'Moisture_%'], 'slope_model')
slip_code  = export_rf_regressor( slip_model,  ['Moisture_%', 'delta_tilt'], 'slip_model')

with open("combined_model.py", "w") as f:
    f.write("# combined_model.py — generated by rf_combined.py\n")
    f.write(f"# Soil CV  : {np.mean(soil_cv)*100:.0f}% (±{np.std(soil_cv)*100:.0f}%)\n")
    f.write(f"# Slope CV : {np.mean(slope_cv)*100:.0f}% (±{np.std(slope_cv)*100:.0f}%)\n")
    f.write(f"# Slip CV R²: {np.mean(slp_cv_r2)*100:.0f}% (±{np.std(slp_cv_r2)*100:.0f}%)\n\n")
    f.write("# --- SOIL SUITABILITY MODEL ---\n")
    f.write(soil_code + "\n\n")
    f.write("# --- SLOPE RISK MODEL ---\n")
    f.write(slope_code + "\n\n")
    f.write("# --- SLIP COUNT MODEL ---\n")
    f.write(slip_code + "\n")

print("\ncombined_model.py written.")
print(f"Soil CV : {np.mean(soil_cv)*100:.2f}%  Slope CV: {np.mean(slope_cv)*100:.2f}%  Slip R²: {np.mean(slp_cv_r2)*100:.2f}%")
