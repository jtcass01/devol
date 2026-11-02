import scipy.stats as st

def calc_95_ci(data):
    n = len(data)
    if n < 2: return 0.0
    return st.sem(data) * st.t.ppf((1 + 0.95) / 2., n - 1)

