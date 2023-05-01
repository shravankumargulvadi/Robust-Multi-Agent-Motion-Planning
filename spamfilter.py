# Importing libraries
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import string

# Read the data
spam_df = pd.read_csv('spam.csv', encoding="ISO-8859-1")
# Create subsets for data
spam_df = spam_df[['v1','v2']]
# Rename the subsets to spam/text
spam_df.rename(columns={'v1': 'spam', 'v2': 'text'}, inplace=True)
# Convert spam column to binary
spam_df.spam = spam_df.spam.apply(lambda s: True if s=='spam' else False)
# Make everything lowercase and remove punctuation
spam_df.text = spam_df.text.apply(lambda t: t.lower().translate(str.maketrans('','',string.punctuation)))
# Make everything random/shuffle the data
spam_df = spam_df.sample(frac=1)

#Test
print(spam_df)