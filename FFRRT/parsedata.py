import csv
import pandas as pd
import numpy as np
# df = pd.read_csv(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a.csv')

df = pd.read_table(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a.csv', delimiter=" ", names=["Frame", "no", "Time", "timestamp","object","num1","num2","val1","val2","val3" ])
# print(df.head(10))
df000 = df[df["num1"] == 000]
df001 = df[df["num1"] == 1]
df002 = df[df["num1"] == 2]
df003 = df[df["num1"] == 3]
df004 = df[df["num1"] == 4]
df005 = df[df["num1"] == 5]
df006 = df[df["num1"] == 6]
df007 = df[df["num1"] == 7]
df0=df000.drop(columns=["Frame", "no", "Time","object","num1","num2" ])
df1=df001.drop(columns=["Frame", "no", "Time","object","num1","num2" ])
df2=df002.drop(columns=["Frame", "no", "Time","object","num1","num2" ])
df3=df003.drop(columns=["Frame", "no", "Time","object","num1","num2" ])
df4=df004.drop(columns=["Frame", "no", "Time","object","num1","num2" ])
df5=df005.drop(columns=["Frame", "no", "Time","object","num1","num2" ])
df6=df006.drop(columns=["Frame", "no", "Time","object","num1","num2" ])
df7=df007.drop(columns=["Frame", "no", "Time","object","num1","num2" ])

arr0=df0.to_numpy()
arr1=df1.to_numpy()
arr2=df2.to_numpy()
arr3=df3.to_numpy()
arr4=df4.to_numpy()
arr5=df5.to_numpy()
arr6=df6.to_numpy()
arr7=df7.to_numpy()
print(np.shape(arr0))
# print(df002.iloc[1,:-1])
# print(df000.iloc[1,:-1])
# print(df001.iloc[1,:-1])
# print(df002.iloc[1,:-1])
# print(df003.iloc[1,:-1])
# print(df004.iloc[1,:-1])
# print(df005.iloc[1,:-1])
# df000.to_csv(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a0.csv', sep='\t')
# df001.to_csv(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a1.csv', sep='\t')
# df002.to_csv(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a2.csv', sep='\t')
# df003.to_csv(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a3.csv', sep='\t')
# df004.to_csv(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a4.csv', sep='\t')
# df005.to_csv(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a5.csv', sep='\t')
# df006.to_csv(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a6.csv', sep='\t')
# df007.to_csv(r'D:\PHD\SEM4\ffrrtsuraj\FFRRT-fast-suraj\2D\a7.csv', sep='\t')


# Frame 0 Time 1761195307305009151 Object 000 000 0.00000 0.00000 0.00000
# Frame 0 Time 1761195307305009151 Object 001 000 0.00000 0.00000 0.00000
# Frame 0 Time 1761195307305009151 Object 002 000 0.00000 0.00000 0.00000
# Frame 0 Time 1761195307305009151 Object 003 000 0.00000 0.00000 0.00000
# Frame 0 Time 1761195307305009151 Object 004 000 0.00000 0.00000 0.00000
# Frame 0 Time 1761195307305009151 Object 005 000 0.00000 0.00000 0.00000
# Frame 0 Time 1761195307305009151 Object 006 000 0.00000 0.00000 0.00000
# Frame 0 Time 1761195307305009151 Object 007 000 0.00000 0.00000 0.00000