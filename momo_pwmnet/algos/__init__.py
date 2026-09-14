# 光流算法包(algos/): algo_common(共享) + algo_dense/algo_dis/algo_sparse/algo_tvl1(可选实现)
# 由主程序按 config.ini [algorithm].mode 动态加载:
#   importlib.import_module("algos.algo_<mode>")