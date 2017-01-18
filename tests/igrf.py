import numpy as np
import matplotlib.pyplot as plt
###################################################################
str = "1900 1905 1910 1915 1920 1925 1930 1935 1940 1945 1950 1955 1960 1965 1970 1975 1980 1985 1990 1995   2000    2005    2010   2015 2020"
years = str.split()
y = []
for i in years:
    y.append(int(i))
#################################################
a = 6371.2 #Earth radius in Kms
latitude = 27.174
longitude = 65.795
t= int(2015)
height = 0. #height in Kilometers
r = height + a  #radial distance in Kilometers
theta = (90.0) - latitude #co-latitude
phi = 0.0
if longitude < 0.0:
    phi = 360.0 + longitude
else:
    phi = longitude;
#################################################

def getPnm(n,m,theta):
    pnm = np.zeros((81,))
    x = np.cos(theta)

    x2 = x * x
    s = np.sqrt(1.0 - x2)

    x3 = x2 * x
    x4 = x3 * x
    x5 = x4 * x
    x6 = x5 * x
    x8 = x6 * x * x
    s2 = s * s
    s3 = s2 * s
    s4 = s3 * s
    s5 = s4 * s
    s6 = s5 * s
    s7 = s6 * s
    s8 = s6 * s

    pnm[0] = 1.0
    pnm[9] = x
    pnm[10] = s
    pnm[18] = 0.5 * ((3.0 * x2) - 1)
    pnm[19] = 3.0 * x * s
    pnm[20] = 3.0 * (1.0 - x2)
    pnm[27] = 0.5 * x * ((5.0 * x2) - 3.0)
    pnm[28] = 1.5 * ((5.0 * x2) - 1.0) * s
    pnm[29] = 15.0 * x * (1.0 - x2)
    pnm[30] = 15.0 * s3
    pnm[36] = 0.125 * ((35.0 * x4) - (30.0 * x2) + 3.0)
    pnm[37] = 2.5 * ((7.0 * x3) - (3.0 * x))
    pnm[38] = 7.5 * (7.0 * x2 - 1) * (1.0 - x2)
    pnm[39] = 105.0 * x * s3
    pnm[40] = 105.0 * s4
    pnm[45] = 0.125 * x * ((63.0 * x4) - (70.0 * x2) + 15.0)
    pnm[46] = 1.875 * s * ((21.0 * x4) - (14.0 * x2) + 1.0)
    pnm[47] = 52.5 * x * (1.0 - x2) * (3.0 * x2 - 1.0)
    pnm[48] = 52.5 * s3 * (9.0 * x2 - 1.0)
    pnm[49] = 945.0 * x * s4
    pnm[50] = 945.0 * s5
    pnm[54] = 0.0625 * ((231.0 * x6) - (315.0 * x4) + (105.0 * x2) - 5.0)
    pnm[55] = 2.625 * x * s * ((33.0 - x4) - (30.0 * x2) + 5.0)
    pnm[56] = 13.125 * s2 * ((33.0 - x4) - (18.0 * x2) + 1.0)
    pnm[57] = 157.5 * x * s3 * (11.0 * x2 - 3.0)
    pnm[58] = 472.5 * s4 * (11.0 * x2 - 1.0)
    pnm[59] = 10395.0 * x * s5
    pnm[60] = 10395.0 * s6
    pnm[63] = 0.625 * x * ((429.0 * x6) - (693.0 * x4) + (315.0 * x2) - 35.0)
    pnm[64] = 0.4375 * s * ((429.0 * x6) - (495.0 * x4) + (135.0 * x2) - 5.0)
    pnm[65] = 7.875 * x * s2 * ((143.0 * x4) - (110.0 * x2) + 15.0)
    pnm[66] = 39.375 * s3 * ((143.0 * x4) - (66.0 * x2) + 3.0)
    pnm[67] = 1732.5 * x * s4 * (13.0 * x2 - 3.0)
    pnm[68] = 5197.5 * s5 * (13.0 * x2 - 1.0)
    pnm[69] = 135135.0 * x * s6
    pnm[70] = 135135.0 * s7
    pnm[72] = 0.0078125 * ((6435.0 * x8) - (12012.0 * x6) + (6930.0 * x4) - (1260.0 * x2) + 35.0)
    pnm[73] = 0.5625 * x * s * ((715.0 * x6) - (1001.0 * x4) + (385.0 * x2) - 35.0)
    pnm[74] = 19.6875 * s2 * ((143.0 * x6) - (143.0 * x4) + (33.0 * x2) - 1.0)
    pnm[75] = 433.125 * x * s3 * ((39.0 * x4) - (26.0 * x2) + 3.0)
    pnm[76] = 1299.375 * s4 * ((65.0 * x4) - (26.0 * x2) + 1.0)
    pnm[77] = 67567.5 * x * s5 * (5.0 * x2 - 1.0)
    pnm[78] = 67567.5 * s6 * (15.0 * x2 - 1.0)
    pnm[79] = 2027025.0 * x * s7
    pnm[80] = 2027025.0 * s8

    return pnm[n*9+m]


#########################################################
def gnmt(n,m,t):
    if t<1900 or t > 2019:
	print "Invalid date."
	exit(1)
    global y
    ret = 0.0
    f = open("samples/igrf12coeffs.txt")
    lines = f.readlines()
    for line in lines:
        fields = line.split()
        if fields[0] == 'g' and int(fields[1]) == n and int(fields[2]) == m:
	    if t >= 2015 :
		ret = float(fields[-1])
	    else:
		for i in range(len(y)):
		    if y[i] <= t and t < (y[i]+5):
			ret = ( float(fields[i+4]) - float(fields[i+3]) )/ 5.0
    f.close()
    return ret
###########################################################################    
def hnmt(n,m,t):
    if t<1900 or t > 2019:
	print "Invalid date."
	exit(1)
    global y
    ret = 0.0
    f = open("samples/igrf12coeffs.txt")
    lines = f.readlines()
    for line in lines:
        fields = line.split()
        if fields[0] == 'h' and int(fields[1]) == n and int(fields[2]) == m:
	    if t >= 2015 :
		ret = float(fields[-1])
	    else:
		for i in range(len(y)):
		    if y[i] <= t and t < (y[i]+5):
			ret = ( float(fields[i+4]) - float(fields[i+3]) )/ 5.0
    f.close()
    return ret
  
####################################################
def computeV(r,theta,phi,t):
    global a
    summation = 0.0
    for n in range(1,9):
        for m in range(n+1):
            summation += (a/r)**(n+1) * (gnmt(n,m,t) * np.cos(m*phi)) * (hnmt(n,m,t) * np.sin(m*phi) * getPnm(n,m,theta))
    return a * summation
####################################################

def computeField():
  global r
  global theta
  global phi
  global t
  
  epsilon = 7./3 - 4./3 -1
  
  #delta = np.sqrt(epsilon) * (abs(theta) * np.sqrt(epsilon))
  delta = 1.0E-10
  Fx = computeV(r,theta,phi,t)
  Fxph = computeV(r,theta+delta,phi,t)
  Fxmh = computeV(r,theta-delta,phi,t)
  derivative = ((Fxph - Fxmh)/(2.*delta))
  factor = (1./r)
  X =  factor * derivative
  print "delta: ", delta
  print "Fx: ", Fx
  print "derivative: ", derivative
  print "factor: ", factor
  print "X: ", X

  
  #delta = np.sqrt(epsilon) * (abs(phi) * np.sqrt(epsilon))
  delta = 1.0E-10
  Fy = computeV(r,theta,phi,t)
  Fyph = computeV(r,theta,phi+delta,t)
  Fymh = computeV(r,theta,phi-delta,t)
  derivative = ((Fyph - Fymh)/(2.*delta))
  factor = (-1./(r*np.sin(theta)))
  Y =  factor * derivative
  print "delta: ", delta
  print "Fy: ", Fy
  print "derivative: ", derivative
  print "factor: ", factor
  print "Y: ", Y
  
  #delta = np.sqrt(epsilon) * (abs(r) * np.sqrt(epsilon))
  delta = 1.0E-10
  Fz = computeV(r,theta,phi,t)
  Fzph = computeV(r+delta,theta,phi,t)
  Fzmh = computeV(r-delta,theta,phi,t)
  derivative = ((Fzph - Fzmh)/(2.*delta))
  factor = 1.
  Z =  factor * derivative
  print "delta: ", delta
  print "Fz: ", Fz
  print "derivative: ", derivative
  print "factor: ", factor
  print "Z: ", Z
  
  H = np.sqrt(X**2 + Y**2)
  F = np.sqrt(X**2 + Y**2 + Z**2)
  D = np.arctan(Y/X)
  I = np.arctan(Z/H)
  
  return H,F,D,I,X,Y,Z

####################################################

H,F,D,I,X,Y,Z = computeField()



print "Input"
print "------"
print "Latitude: ", latitude #, "Radians  ", latitude * (180./np.pi), " Degrees" 
print "Longitude: ", longitude #, "Radians  ", longitude * (180./np.pi), " Degrees" 
print "Height: ", height, "Kilometers  "
print "Year: ", t 

print ""

print "Output"
print "------"
print "H: ", H
print "F: ", F
print "D: ", D #, "Radians  ", D * (180./np.pi), " Degrees" 
print "I: ", I #, "Radians  ", I * (180./np.pi), " Degrees" 
print "X: ", X
print "Z: ", Z
print "Y: ", Y






