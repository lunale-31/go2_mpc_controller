import pandas as pd

full_dump = pd.read_csv("full-dump.csv")
full_dump.drop(full_dump.columns[
    full_dump.columns.str.contains('unnamed', case=False)
], axis=1, inplace=True)

frames = {
    0: "fast-fw",
    1910: "fast-bw",
    4147: "medium-fw",
    7549: "medium-bw",
    11163: "slow-fw",
    19648: "slow-bw"
}

keys = sorted(list(frames.keys()))
for i, key in enumerate(keys):
    if i + 1 == len(keys):
        split: pd.DataFrame = full_dump.iloc[key:]
    else:
        split: pd.DataFrame = full_dump.iloc[key:keys[i+1]]
    
    split.to_csv(f"split-{frames[key]}.csv")
    
