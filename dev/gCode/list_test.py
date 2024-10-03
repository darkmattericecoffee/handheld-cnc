import pandas as pd

# Example DataFrame
segments_df = pd.DataFrame(columns=["p0"])

# Assigning lists to the "p0" column
segments_df.at[0, "p0"] = [1, 2, 3]
segments_df.at[1, "p0"] = [4, 5, 6]

# Check the column's data type
print(segments_df["p0"].dtype)  # Should print "object"

# Display the DataFrame
print(segments_df)