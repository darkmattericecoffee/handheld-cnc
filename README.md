# handheld-cnc
A handheld CNC router project.

# Debugging
To produce a debug logFile to use with `animate.ipynb`, make sure the variable `outputMode` is set to `1`. Then run the following code in a **platformIO terminal**:
```
pio device mointor > /logFiles/logFile_XX.txt
```