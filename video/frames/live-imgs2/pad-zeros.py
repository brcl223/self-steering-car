#!/usr/bin/env python
import sys,os,re

for file in sys.argv[1:]:
    if __file__ in file: continue
    words = re.split('_|\.',file)
    words[-2] = words[-2].zfill(4)
    new_name = "_".join(words[:-1]) + "." + words[-1]
    os.rename(file,new_name)
