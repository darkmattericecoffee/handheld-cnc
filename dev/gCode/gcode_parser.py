import re
import pandas as pd
from gcodeparser import GcodeParser

# open gcode file and store contents as variable
with open('handheld-cnc/dev/gcode/basePlate_test.nc', 'r') as f:
  gcode = f.read()
  
# parsedNC = GcodeParser(gcode).lines    # get parsed gcode lines
df = pd.DataFrame(GcodeParser(gcode).lines)

print(df.iloc[])

# print("Y coordinates:", y)
# print("Z coordinates:", z)
