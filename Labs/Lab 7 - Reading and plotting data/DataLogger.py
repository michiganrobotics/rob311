"""
Data Logger for AKP2 Project
Daniel Gonzalez
dgonz@mit.edu
"""

import numpy

class dataLogger():
    myData = []

    def __init__(self, name):
        self.name = name
        f = open(name, 'w')
    
    def writeOut(self):
        print('Storing data...\n')
        f = open(self.name, 'a')
        outTxt = []
        for data in self.myData:
            for e in data:
                outTxt.append(str(e) + ' ')
            outTxt.append('\n')
        f.write(''.join(outTxt))
        f.write('\n')
        self.myData = []
        #numpy.savetxt(name,self.myData)

    def appendData(self,val):
        self.myData.append(val)
