import pickle

with open("test.csv", 'rb') as f:
    a = pickle.load(f)

print(a)
