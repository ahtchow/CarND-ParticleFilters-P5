"""
 Localization is nothing else then an iteration of:

       GAIN(INFO) SENSE <-> MOVE (LOSE INFO)
    
    Theres a measure of info called entropy:
         - summation( p(Xi)*logp(Xi))

 OVERALL: 
    BELIEF = PROBABILITY
    SENSE = PRODUCT FOLLOW BY NORMALIZATION
    MOVE = CONVOLUTION (ADDITION)

 Formal Definition of Probability:
    0 <= P(X) <= 1
"""
#Given the list motions=[1,1] which means the robot 
#moves right and then right again, compute the posterior 
#distribution if the robot first senses red, then moves 
#right one, then senses green, then moves right again, 
#starting with a uniform prior distribution.

p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
motions = [1,1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q

def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U) % len(p)]
        s = s + pOvershoot * p[(i-U-1) % len(p)]
        s = s + pUndershoot * p[(i-U+1) % len(p)]
        q.append(s)
    return q

# SENSE, UPDATE, MOVE, UPDATE
for i in range(len(measurements)):
    p = sense(p, measurements[i]) # GAIN UNCERTAINTY
    p = move(p, 1) # LOSE UNCERTAINTY

print (p)         

"""
MOTION - TOTAL PROBABILITY

    P(Xi/t) = Summation(P(Xj/t-1) * P(Xi|Xj))
    or
    P(A) = Summation(P(A|B)*P(B))  (CONVOLUTION)


"""