import matplotlib as plt
import pandas as pd
import matplotlib.pyplot as plt

path_to_file = '/Users/sebastian/Documents/entropy_values.csv'


df = pd.read_csv(path_to_file)

df.index = df.index*5 #needed this for entropy since period was 5 seconds, you might not need this. 

plt.plot(df)
plt.ylabel('Entropy')
plt.xlabel('Seconds')
plt.title('Entropy for Base line and WFD algorithms')
plt.show()
