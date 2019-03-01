
import math

def quatConj(q):
    conj = [0,0,0,0]
    conj[0] = q[0]
    conj[1] = -q[1]
    conj[2] = -q[2]
    conj[3] = -q[3]
    return conj

def quatProduct(q1, q2):
    prod = [0,0,0,0]
    prod[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    prod[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    prod[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    prod[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return prod

def quatDist(q1, q2):
    dotProduct = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]

    # Sometimes dotProduct is very slightly over 1
    dotProduct = min(dotProduct, 1)

    return 2*math.acos(dotProduct)